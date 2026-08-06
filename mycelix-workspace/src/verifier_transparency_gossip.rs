// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Temporal gossip consistency and witness-set rotation for verifier transparency.
//!
//! Witness cosigning detects a same-size split view at one instant. This module
//! requires authenticated consistency proofs across time and monotonic witness
//! rotation. Opaque verified tokens—not caller supplied booleans—cross the
//! cryptographic boundary.

use crate::error::{ZkpError, ZkpResult};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const VERIFIER_TRANSPARENCY_GOSSIP_POLICY_ID: &str =
    "mycelix.proof.verifier-transparency-gossip.policy.v1";
pub const VERIFIER_TRANSPARENCY_GOSSIP_POLICY_VERSION: u16 = 1;
pub const VERIFIER_TRANSPARENCY_CONSISTENCY_LINK_ID: &str =
    "mycelix.proof.verifier-transparency-gossip.consistency-link.v1";
pub const VERIFIER_TRANSPARENCY_CONSISTENCY_LINK_VERSION: u16 = 1;
pub const VERIFIER_TRANSPARENCY_WITNESS_ROTATION_ID: &str =
    "mycelix.proof.verifier-transparency-gossip.witness-rotation.v1";
pub const VERIFIER_TRANSPARENCY_WITNESS_ROTATION_VERSION: u16 = 1;
pub const VERIFIER_TRANSPARENCY_TEMPORAL_CONFLICT_ID: &str =
    "mycelix.proof.verifier-transparency-gossip.temporal-conflict.v1";
pub const VERIFIER_TRANSPARENCY_TEMPORAL_CONFLICT_VERSION: u16 = 1;
pub const VERIFIER_TRANSPARENCY_GOSSIP_RECOVERY_ID: &str =
    "mycelix.proof.verifier-transparency-gossip.recovery.v1";
pub const VERIFIER_TRANSPARENCY_GOSSIP_RECOVERY_VERSION: u16 = 1;
pub const VERIFIER_TRANSPARENCY_GOSSIP_CHECKPOINT_ID: &str =
    "mycelix.proof.verifier-transparency-gossip.checkpoint.v1";
pub const VERIFIER_TRANSPARENCY_GOSSIP_CHECKPOINT_VERSION: u16 = 1;

pub const MAX_TRANSPARENCY_CONSISTENCY_PROOF_BYTES: usize = 256 * 1024;
pub const MAX_TRANSPARENCY_GOSSIP_AGE_SECS: u64 = 24 * 60 * 60;

const POLICY_DOMAIN: &[u8] = b"MYCELIX:Proof:TransparencyGossip:Policy:v1";
const LINK_DOMAIN: &[u8] = b"MYCELIX:Proof:TransparencyGossip:ConsistencyLink:v1";
const ROTATION_DOMAIN: &[u8] = b"MYCELIX:Proof:TransparencyGossip:WitnessRotation:v1";
const CONFLICT_DOMAIN: &[u8] = b"MYCELIX:Proof:TransparencyGossip:TemporalConflict:v1";
const RECOVERY_DOMAIN: &[u8] = b"MYCELIX:Proof:TransparencyGossip:Recovery:v1";
const CHECKPOINT_DOMAIN: &[u8] = b"MYCELIX:Proof:TransparencyGossip:Checkpoint:v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierTransparencyGossipPolicy {
    pub protocol_id: String,
    pub protocol_version: u16,
    pub maximum_link_age_secs: u64,
    pub require_cryptographic_consistency_proof: bool,
    pub require_dual_quorum_witness_rotation: bool,
    pub forbid_threshold_reduction: bool,
    pub require_operator_independence: bool,
    pub require_external_checkpoint: bool,
}

impl VerifierTransparencyGossipPolicy {
    pub fn validate(&self) -> ZkpResult<()> {
        if self.protocol_id != VERIFIER_TRANSPARENCY_GOSSIP_POLICY_ID
            || self.protocol_version != VERIFIER_TRANSPARENCY_GOSSIP_POLICY_VERSION
            || self.maximum_link_age_secs == 0
            || self.maximum_link_age_secs > MAX_TRANSPARENCY_GOSSIP_AGE_SECS
            || !self.require_cryptographic_consistency_proof
            || !self.require_dual_quorum_witness_rotation
            || !self.forbid_threshold_reduction
            || !self.require_operator_independence
            || !self.require_external_checkpoint
        {
            return Err(ZkpError::InvalidMetadata(
                "invalid transparency gossip policy".into(),
            ));
        }
        Ok(())
    }

    pub fn canonical_hash(&self) -> ZkpResult<[u8; 32]> {
        self.validate()?;
        let mut hasher = Sha256::new();
        hash_field(&mut hasher, POLICY_DOMAIN);
        hash_field(&mut hasher, self.protocol_id.as_bytes());
        hasher.update(self.protocol_version.to_le_bytes());
        hasher.update(self.maximum_link_age_secs.to_le_bytes());
        hasher.update([self.require_cryptographic_consistency_proof as u8]);
        hasher.update([self.require_dual_quorum_witness_rotation as u8]);
        hasher.update([self.forbid_threshold_reduction as u8]);
        hasher.update([self.require_operator_independence as u8]);
        hasher.update([self.require_external_checkpoint as u8]);
        Ok(digest_array(hasher.finalize()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierTransparencyConsistencyLink {
    pub protocol_id: String,
    pub protocol_version: u16,
    pub log_id: [u8; 32],
    pub prior_tree_size: u64,
    pub prior_tree_root: [u8; 32],
    pub next_tree_size: u64,
    pub next_tree_root: [u8; 32],
    pub consistency_proof: Vec<u8>,
    pub observed_at: u64,
}

impl VerifierTransparencyConsistencyLink {
    pub fn validate_shape(
        &self,
        policy: &VerifierTransparencyGossipPolicy,
        now: u64,
    ) -> ZkpResult<()> {
        policy.validate()?;
        if self.protocol_id != VERIFIER_TRANSPARENCY_CONSISTENCY_LINK_ID
            || self.protocol_version != VERIFIER_TRANSPARENCY_CONSISTENCY_LINK_VERSION
            || self.log_id == [0; 32]
            || self.prior_tree_size == 0
            || self.next_tree_size <= self.prior_tree_size
            || self.prior_tree_root == [0; 32]
            || self.next_tree_root == [0; 32]
            || self.consistency_proof.is_empty()
            || self.consistency_proof.len() > MAX_TRANSPARENCY_CONSISTENCY_PROOF_BYTES
            || self.observed_at > now
            || now.saturating_sub(self.observed_at) > policy.maximum_link_age_secs
        {
            return Err(ZkpError::InvalidMetadata(
                "invalid or stale transparency consistency link".into(),
            ));
        }
        Ok(())
    }

    pub fn canonical_hash(&self) -> ZkpResult<[u8; 32]> {
        if self.protocol_id != VERIFIER_TRANSPARENCY_CONSISTENCY_LINK_ID
            || self.protocol_version != VERIFIER_TRANSPARENCY_CONSISTENCY_LINK_VERSION
        {
            return Err(ZkpError::InvalidMetadata(
                "invalid transparency consistency-link protocol".into(),
            ));
        }
        let mut hasher = Sha256::new();
        hash_field(&mut hasher, LINK_DOMAIN);
        hash_field(&mut hasher, self.protocol_id.as_bytes());
        hasher.update(self.protocol_version.to_le_bytes());
        hasher.update(self.log_id);
        hasher.update(self.prior_tree_size.to_le_bytes());
        hasher.update(self.prior_tree_root);
        hasher.update(self.next_tree_size.to_le_bytes());
        hasher.update(self.next_tree_root);
        hash_field(&mut hasher, &self.consistency_proof);
        hasher.update(self.observed_at.to_le_bytes());
        Ok(digest_array(hasher.finalize()))
    }
}

pub trait VerifierTransparencyConsistencyProofVerifier {
    fn verify_consistency_proof(&self, link: &VerifierTransparencyConsistencyLink) -> bool;
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedVerifierTransparencyConsistencyLink {
    link: VerifierTransparencyConsistencyLink,
    link_hash: [u8; 32],
}

impl VerifiedVerifierTransparencyConsistencyLink {
    pub fn link(&self) -> &VerifierTransparencyConsistencyLink {
        &self.link
    }

    pub fn link_hash(&self) -> [u8; 32] {
        self.link_hash
    }
}

pub fn verify_verifier_transparency_consistency_link(
    policy: &VerifierTransparencyGossipPolicy,
    link: &VerifierTransparencyConsistencyLink,
    verifier: &impl VerifierTransparencyConsistencyProofVerifier,
    now: u64,
) -> ZkpResult<VerifiedVerifierTransparencyConsistencyLink> {
    link.validate_shape(policy, now)?;
    if !verifier.verify_consistency_proof(link) {
        return Err(ZkpError::VerificationFailed(
            "transparency consistency proof verification failed".into(),
        ));
    }
    Ok(VerifiedVerifierTransparencyConsistencyLink {
        link: link.clone(),
        link_hash: link.canonical_hash()?,
    })
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierTransparencyWitnessSetRotation {
    pub protocol_id: String,
    pub protocol_version: u16,
    pub previous_witness_set_hash: [u8; 32],
    pub previous_witness_set_epoch: u64,
    pub previous_threshold: u16,
    pub previous_operator_count: u16,
    pub next_witness_set_hash: [u8; 32],
    pub next_witness_set_epoch: u64,
    pub next_threshold: u16,
    pub next_operator_count: u16,
    pub predecessor_quorum_hash: [u8; 32],
    pub successor_quorum_hash: [u8; 32],
    pub issued_at: u64,
}

impl VerifierTransparencyWitnessSetRotation {
    pub fn validate(&self) -> ZkpResult<()> {
        if self.protocol_id != VERIFIER_TRANSPARENCY_WITNESS_ROTATION_ID
            || self.protocol_version != VERIFIER_TRANSPARENCY_WITNESS_ROTATION_VERSION
            || self.previous_witness_set_hash == [0; 32]
            || self.next_witness_set_hash == [0; 32]
            || self.previous_witness_set_hash == self.next_witness_set_hash
            || self.previous_witness_set_epoch == 0
            || self.next_witness_set_epoch != self.previous_witness_set_epoch.saturating_add(1)
            || self.previous_threshold < 2
            || self.next_threshold < self.previous_threshold
            || self.previous_operator_count < 2
            || self.next_operator_count < 2
            || self.next_operator_count < self.next_threshold
            || self.predecessor_quorum_hash == [0; 32]
            || self.successor_quorum_hash == [0; 32]
            || self.issued_at == 0
        {
            return Err(ZkpError::InvalidMetadata(
                "invalid transparency witness-set rotation".into(),
            ));
        }
        Ok(())
    }

    pub fn canonical_hash(&self) -> ZkpResult<[u8; 32]> {
        self.validate()?;
        let mut hasher = Sha256::new();
        hash_field(&mut hasher, ROTATION_DOMAIN);
        hash_field(&mut hasher, self.protocol_id.as_bytes());
        hasher.update(self.protocol_version.to_le_bytes());
        hasher.update(self.previous_witness_set_hash);
        hasher.update(self.previous_witness_set_epoch.to_le_bytes());
        hasher.update(self.previous_threshold.to_le_bytes());
        hasher.update(self.previous_operator_count.to_le_bytes());
        hasher.update(self.next_witness_set_hash);
        hasher.update(self.next_witness_set_epoch.to_le_bytes());
        hasher.update(self.next_threshold.to_le_bytes());
        hasher.update(self.next_operator_count.to_le_bytes());
        hasher.update(self.predecessor_quorum_hash);
        hasher.update(self.successor_quorum_hash);
        hasher.update(self.issued_at.to_le_bytes());
        Ok(digest_array(hasher.finalize()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierTransparencyTemporalConflictEvidence {
    pub protocol_id: String,
    pub protocol_version: u16,
    pub log_id: [u8; 32],
    pub common_prior_tree_size: u64,
    pub common_prior_tree_root: [u8; 32],
    pub first_link_hash: [u8; 32],
    pub second_link_hash: [u8; 32],
    pub detected_at: u64,
}

impl VerifierTransparencyTemporalConflictEvidence {
    pub fn canonical_hash(&self) -> ZkpResult<[u8; 32]> {
        if self.protocol_id != VERIFIER_TRANSPARENCY_TEMPORAL_CONFLICT_ID
            || self.protocol_version != VERIFIER_TRANSPARENCY_TEMPORAL_CONFLICT_VERSION
            || self.log_id == [0; 32]
            || self.common_prior_tree_size == 0
            || self.common_prior_tree_root == [0; 32]
            || self.first_link_hash == [0; 32]
            || self.second_link_hash == [0; 32]
            || self.first_link_hash >= self.second_link_hash
            || self.detected_at == 0
        {
            return Err(ZkpError::InvalidMetadata(
                "invalid temporal transparency conflict evidence".into(),
            ));
        }
        let mut hasher = Sha256::new();
        hash_field(&mut hasher, CONFLICT_DOMAIN);
        hasher.update(self.log_id);
        hasher.update(self.common_prior_tree_size.to_le_bytes());
        hasher.update(self.common_prior_tree_root);
        hasher.update(self.first_link_hash);
        hasher.update(self.second_link_hash);
        hasher.update(self.detected_at.to_le_bytes());
        Ok(digest_array(hasher.finalize()))
    }
}

pub fn detect_verifier_transparency_temporal_conflict(
    first: &VerifiedVerifierTransparencyConsistencyLink,
    second: &VerifiedVerifierTransparencyConsistencyLink,
    detected_at: u64,
) -> ZkpResult<Option<VerifierTransparencyTemporalConflictEvidence>> {
    let a = first.link();
    let b = second.link();
    if a.log_id != b.log_id
        || a.prior_tree_size != b.prior_tree_size
        || a.prior_tree_root != b.prior_tree_root
        || (a.next_tree_size == b.next_tree_size && a.next_tree_root == b.next_tree_root)
    {
        return Ok(None);
    }
    let (first_link_hash, second_link_hash) = if first.link_hash() < second.link_hash() {
        (first.link_hash(), second.link_hash())
    } else {
        (second.link_hash(), first.link_hash())
    };
    Ok(Some(VerifierTransparencyTemporalConflictEvidence {
        protocol_id: VERIFIER_TRANSPARENCY_TEMPORAL_CONFLICT_ID.into(),
        protocol_version: VERIFIER_TRANSPARENCY_TEMPORAL_CONFLICT_VERSION,
        log_id: a.log_id,
        common_prior_tree_size: a.prior_tree_size,
        common_prior_tree_root: a.prior_tree_root,
        first_link_hash,
        second_link_hash,
        detected_at,
    }))
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierTransparencyGossipRecovery {
    pub protocol_id: String,
    pub protocol_version: u16,
    pub conflict_hash: [u8; 32],
    pub replacement_link_hash: [u8; 32],
    pub replacement_tree_size: u64,
    pub recovery_authority_set_hash: [u8; 32],
    pub recovery_authority_epoch: u64,
    pub issued_at: u64,
}

pub fn verify_verifier_transparency_gossip_recovery(
    conflict: &VerifierTransparencyTemporalConflictEvidence,
    replacement: &VerifiedVerifierTransparencyConsistencyLink,
    recovery: &VerifierTransparencyGossipRecovery,
) -> ZkpResult<()> {
    let conflict_hash = conflict.canonical_hash()?;
    let replacement_link = replacement.link();
    if recovery.protocol_id != VERIFIER_TRANSPARENCY_GOSSIP_RECOVERY_ID
        || recovery.protocol_version != VERIFIER_TRANSPARENCY_GOSSIP_RECOVERY_VERSION
        || recovery.conflict_hash != conflict_hash
        || recovery.replacement_link_hash != replacement.link_hash()
        || recovery.replacement_tree_size != replacement_link.next_tree_size
        || recovery.replacement_tree_size <= conflict.common_prior_tree_size
        || recovery.recovery_authority_set_hash == [0; 32]
        || recovery.recovery_authority_epoch == 0
        || recovery.issued_at < conflict.detected_at
    {
        return Err(ZkpError::VerificationFailed(
            "invalid transparency gossip recovery".into(),
        ));
    }
    Ok(())
}

impl VerifierTransparencyGossipRecovery {
    pub fn canonical_hash(&self) -> ZkpResult<[u8; 32]> {
        if self.protocol_id != VERIFIER_TRANSPARENCY_GOSSIP_RECOVERY_ID
            || self.protocol_version != VERIFIER_TRANSPARENCY_GOSSIP_RECOVERY_VERSION
        {
            return Err(ZkpError::InvalidMetadata(
                "invalid transparency gossip recovery protocol".into(),
            ));
        }
        let mut hasher = Sha256::new();
        hash_field(&mut hasher, RECOVERY_DOMAIN);
        hasher.update(self.conflict_hash);
        hasher.update(self.replacement_link_hash);
        hasher.update(self.replacement_tree_size.to_le_bytes());
        hasher.update(self.recovery_authority_set_hash);
        hasher.update(self.recovery_authority_epoch.to_le_bytes());
        hasher.update(self.issued_at.to_le_bytes());
        Ok(digest_array(hasher.finalize()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierTransparencyGossipCheckpoint {
    pub protocol_id: String,
    pub protocol_version: u16,
    pub checkpoint_sequence: u64,
    pub previous_checkpoint_hash: Option<[u8; 32]>,
    pub gossip_policy_hash: [u8; 32],
    pub active_witness_set_hash: [u8; 32],
    pub active_witness_set_epoch: u64,
    pub latest_consistency_link_hash: [u8; 32],
    pub latest_tree_size: u64,
    pub recovery_hash: Option<[u8; 32]>,
    pub issued_at: u64,
}

impl VerifierTransparencyGossipCheckpoint {
    pub fn canonical_hash(&self) -> ZkpResult<[u8; 32]> {
        if self.protocol_id != VERIFIER_TRANSPARENCY_GOSSIP_CHECKPOINT_ID
            || self.protocol_version != VERIFIER_TRANSPARENCY_GOSSIP_CHECKPOINT_VERSION
            || self.checkpoint_sequence == 0
            || self.gossip_policy_hash == [0; 32]
            || self.active_witness_set_hash == [0; 32]
            || self.active_witness_set_epoch == 0
            || self.latest_consistency_link_hash == [0; 32]
            || self.latest_tree_size == 0
            || self.issued_at == 0
        {
            return Err(ZkpError::InvalidMetadata(
                "invalid transparency gossip checkpoint".into(),
            ));
        }
        let mut hasher = Sha256::new();
        hash_field(&mut hasher, CHECKPOINT_DOMAIN);
        hasher.update(self.checkpoint_sequence.to_le_bytes());
        match self.previous_checkpoint_hash {
            Some(hash) => {
                hasher.update([1]);
                hasher.update(hash);
            }
            None => hasher.update([0]),
        }
        hasher.update(self.gossip_policy_hash);
        hasher.update(self.active_witness_set_hash);
        hasher.update(self.active_witness_set_epoch.to_le_bytes());
        hasher.update(self.latest_consistency_link_hash);
        hasher.update(self.latest_tree_size.to_le_bytes());
        match self.recovery_hash {
            Some(hash) => {
                hasher.update([1]);
                hasher.update(hash);
            }
            None => hasher.update([0]),
        }
        hasher.update(self.issued_at.to_le_bytes());
        Ok(digest_array(hasher.finalize()))
    }
}

pub fn validate_verifier_transparency_gossip_checkpoint_successor(
    previous: &VerifierTransparencyGossipCheckpoint,
    next: &VerifierTransparencyGossipCheckpoint,
) -> ZkpResult<()> {
    if next.previous_checkpoint_hash != Some(previous.canonical_hash()?)
        || next.checkpoint_sequence != previous.checkpoint_sequence.saturating_add(1)
        || next.gossip_policy_hash != previous.gossip_policy_hash
        || next.active_witness_set_epoch < previous.active_witness_set_epoch
        || next.latest_tree_size < previous.latest_tree_size
        || next.issued_at < previous.issued_at
    {
        return Err(ZkpError::VerificationFailed(
            "transparency gossip checkpoint rollback or discontinuity".into(),
        ));
    }
    Ok(())
}

fn hash_field(hasher: &mut Sha256, value: &[u8]) {
    hasher.update((value.len() as u64).to_le_bytes());
    hasher.update(value);
}

fn digest_array(value: impl AsRef<[u8]>) -> [u8; 32] {
    let mut out = [0u8; 32];
    out.copy_from_slice(value.as_ref());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Accept;
    impl VerifierTransparencyConsistencyProofVerifier for Accept {
        fn verify_consistency_proof(&self, _: &VerifierTransparencyConsistencyLink) -> bool {
            true
        }
    }

    fn policy() -> VerifierTransparencyGossipPolicy {
        VerifierTransparencyGossipPolicy {
            protocol_id: VERIFIER_TRANSPARENCY_GOSSIP_POLICY_ID.into(),
            protocol_version: VERIFIER_TRANSPARENCY_GOSSIP_POLICY_VERSION,
            maximum_link_age_secs: 600,
            require_cryptographic_consistency_proof: true,
            require_dual_quorum_witness_rotation: true,
            forbid_threshold_reduction: true,
            require_operator_independence: true,
            require_external_checkpoint: true,
        }
    }

    fn link(root: u8, size: u64) -> VerifierTransparencyConsistencyLink {
        VerifierTransparencyConsistencyLink {
            protocol_id: VERIFIER_TRANSPARENCY_CONSISTENCY_LINK_ID.into(),
            protocol_version: VERIFIER_TRANSPARENCY_CONSISTENCY_LINK_VERSION,
            log_id: [1; 32],
            prior_tree_size: 100,
            prior_tree_root: [2; 32],
            next_tree_size: size,
            next_tree_root: [root; 32],
            consistency_proof: vec![9; 32],
            observed_at: 1_000,
        }
    }

    #[test]
    fn stale_or_rollback_links_fail_closed() {
        assert!(verify_verifier_transparency_consistency_link(
            &policy(),
            &link(3, 101),
            &Accept,
            1_500,
        )
        .is_ok());
        assert!(verify_verifier_transparency_consistency_link(
            &policy(),
            &link(3, 101),
            &Accept,
            1_700,
        )
        .is_err());
        assert!(verify_verifier_transparency_consistency_link(
            &policy(),
            &link(3, 99),
            &Accept,
            1_100,
        )
        .is_err());
    }

    #[test]
    fn witness_rotation_rejects_epoch_skip_and_threshold_reduction() {
        let valid = VerifierTransparencyWitnessSetRotation {
            protocol_id: VERIFIER_TRANSPARENCY_WITNESS_ROTATION_ID.into(),
            protocol_version: VERIFIER_TRANSPARENCY_WITNESS_ROTATION_VERSION,
            previous_witness_set_hash: [1; 32],
            previous_witness_set_epoch: 1,
            previous_threshold: 2,
            previous_operator_count: 2,
            next_witness_set_hash: [2; 32],
            next_witness_set_epoch: 2,
            next_threshold: 2,
            next_operator_count: 2,
            predecessor_quorum_hash: [3; 32],
            successor_quorum_hash: [4; 32],
            issued_at: 1,
        };
        assert!(valid.validate().is_ok());
        let mut skipped = valid.clone();
        skipped.next_witness_set_epoch = 3;
        assert!(skipped.validate().is_err());
        let mut reduced = valid;
        reduced.next_threshold = 1;
        assert!(reduced.validate().is_err());
    }

    #[test]
    fn temporal_conflict_evidence_is_order_independent() {
        let first = verify_verifier_transparency_consistency_link(
            &policy(),
            &link(3, 101),
            &Accept,
            1_100,
        )
        .unwrap();
        let second = verify_verifier_transparency_consistency_link(
            &policy(),
            &link(4, 102),
            &Accept,
            1_100,
        )
        .unwrap();
        let ab = detect_verifier_transparency_temporal_conflict(&first, &second, 1_200)
            .unwrap()
            .unwrap();
        let ba = detect_verifier_transparency_temporal_conflict(&second, &first, 1_200)
            .unwrap()
            .unwrap();
        assert_eq!(ab.canonical_hash().unwrap(), ba.canonical_hash().unwrap());
    }
}
