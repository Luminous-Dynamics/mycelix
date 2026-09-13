//! Append-only transparency and independent witness contracts for Mycelix elections.
//!
//! This crate is deliberately proof-backend neutral. It does not implement a
//! Merkle tree, signature scheme, gossip transport, or Holochain storage path.
//! It defines the exact checkpoint lineage and witness-independence invariants
//! those implementations must satisfy.

use std::collections::BTreeSet;

use election_integrity_types::{Digest32, PUBLIC_ELECTION_PROFILE_ID};
use serde::{Deserialize, Serialize};

pub const TRANSPARENCY_PROFILE_ID: &str = "mycelix-public-election-transparency-v1";
pub const WITNESS_PROFILE_ID: &str = "mycelix-public-election-witness-quorum-v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ElectionTransparencyCheckpointV1 {
    pub public_election_profile_id: String,
    pub transparency_profile_id: String,
    pub election_constitution_digest: Digest32,
    pub log_parameters_digest: Digest32,
    pub canonicalization_profile_digest: Digest32,
    pub checkpoint_sequence: u64,
    pub tree_size: u64,
    pub root_digest: Digest32,
    pub previous_checkpoint_digest: Option<Digest32>,
    pub consistency_proof_digest: Option<Digest32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckpointViolation {
    WrongPublicElectionProfile,
    WrongTransparencyProfile,
    ZeroElectionConstitutionDigest,
    ZeroLogParametersDigest,
    ZeroCanonicalizationProfileDigest,
    ZeroRootDigest,
    GenesisHasPredecessor,
    GenesisHasConsistencyProof,
    SuccessorMissingPredecessor,
    SuccessorMissingConsistencyProof,
    ZeroPredecessorDigest,
    ZeroConsistencyProofDigest,
}

pub fn validate_checkpoint_structure(
    checkpoint: &ElectionTransparencyCheckpointV1,
) -> Result<(), CheckpointViolation> {
    if checkpoint.public_election_profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(CheckpointViolation::WrongPublicElectionProfile);
    }
    if checkpoint.transparency_profile_id != TRANSPARENCY_PROFILE_ID {
        return Err(CheckpointViolation::WrongTransparencyProfile);
    }
    let zero = [0_u8; 32];
    if checkpoint.election_constitution_digest == zero {
        return Err(CheckpointViolation::ZeroElectionConstitutionDigest);
    }
    if checkpoint.log_parameters_digest == zero {
        return Err(CheckpointViolation::ZeroLogParametersDigest);
    }
    if checkpoint.canonicalization_profile_digest == zero {
        return Err(CheckpointViolation::ZeroCanonicalizationProfileDigest);
    }
    if checkpoint.root_digest == zero {
        return Err(CheckpointViolation::ZeroRootDigest);
    }

    if checkpoint.checkpoint_sequence == 0 {
        if checkpoint.previous_checkpoint_digest.is_some() {
            return Err(CheckpointViolation::GenesisHasPredecessor);
        }
        if checkpoint.consistency_proof_digest.is_some() {
            return Err(CheckpointViolation::GenesisHasConsistencyProof);
        }
    } else {
        let previous = checkpoint
            .previous_checkpoint_digest
            .ok_or(CheckpointViolation::SuccessorMissingPredecessor)?;
        if previous == zero {
            return Err(CheckpointViolation::ZeroPredecessorDigest);
        }
        let consistency = checkpoint
            .consistency_proof_digest
            .ok_or(CheckpointViolation::SuccessorMissingConsistencyProof)?;
        if consistency == zero {
            return Err(CheckpointViolation::ZeroConsistencyProofDigest);
        }
    }

    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckpointSuccessorViolation {
    Previous(CheckpointViolation),
    Next(CheckpointViolation),
    WrongElectionConstitution,
    LogParametersChanged,
    CanonicalizationProfileChanged,
    SequenceNotContiguous,
    TreeDidNotGrow,
    PredecessorDigestMismatch,
}

pub fn validate_checkpoint_successor(
    previous: &ElectionTransparencyCheckpointV1,
    previous_checkpoint_digest: Digest32,
    next: &ElectionTransparencyCheckpointV1,
) -> Result<(), CheckpointSuccessorViolation> {
    validate_checkpoint_structure(previous).map_err(CheckpointSuccessorViolation::Previous)?;
    validate_checkpoint_structure(next).map_err(CheckpointSuccessorViolation::Next)?;

    if previous.election_constitution_digest != next.election_constitution_digest {
        return Err(CheckpointSuccessorViolation::WrongElectionConstitution);
    }
    if previous.log_parameters_digest != next.log_parameters_digest {
        return Err(CheckpointSuccessorViolation::LogParametersChanged);
    }
    if previous.canonicalization_profile_digest != next.canonicalization_profile_digest {
        return Err(CheckpointSuccessorViolation::CanonicalizationProfileChanged);
    }
    if next.checkpoint_sequence != previous.checkpoint_sequence + 1 {
        return Err(CheckpointSuccessorViolation::SequenceNotContiguous);
    }
    if next.tree_size <= previous.tree_size {
        return Err(CheckpointSuccessorViolation::TreeDidNotGrow);
    }
    if next.previous_checkpoint_digest != Some(previous_checkpoint_digest) {
        return Err(CheckpointSuccessorViolation::PredecessorDigestMismatch);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CheckpointRefV1 {
    pub checkpoint_digest: Digest32,
    pub election_constitution_digest: Digest32,
    pub checkpoint_sequence: u64,
    pub tree_size: u64,
    pub root_digest: Digest32,
    pub previous_checkpoint_digest: Option<Digest32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckpointPairClassification {
    SameCheckpoint,
    DifferentSequence,
    EquivocatingSuccessors,
    DivergentHistory,
}

pub fn classify_checkpoint_pair(
    first: &CheckpointRefV1,
    second: &CheckpointRefV1,
) -> CheckpointPairClassification {
    if first.checkpoint_digest == second.checkpoint_digest
        && first.election_constitution_digest == second.election_constitution_digest
        && first.checkpoint_sequence == second.checkpoint_sequence
        && first.tree_size == second.tree_size
        && first.root_digest == second.root_digest
        && first.previous_checkpoint_digest == second.previous_checkpoint_digest
    {
        return CheckpointPairClassification::SameCheckpoint;
    }
    if first.checkpoint_sequence != second.checkpoint_sequence {
        return CheckpointPairClassification::DifferentSequence;
    }
    if first.election_constitution_digest == second.election_constitution_digest
        && first.previous_checkpoint_digest == second.previous_checkpoint_digest
    {
        return CheckpointPairClassification::EquivocatingSuccessors;
    }
    CheckpointPairClassification::DivergentHistory
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InclusionEvidenceRefV1 {
    pub checkpoint_digest: Digest32,
    pub tree_size: u64,
    pub leaf_index: u64,
    pub leaf_digest: Digest32,
    pub inclusion_proof_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InclusionEvidenceViolation {
    ZeroCheckpointDigest,
    EmptyTree,
    LeafIndexOutOfRange,
    ZeroLeafDigest,
    ZeroInclusionProofDigest,
}

pub fn validate_inclusion_evidence_ref(
    evidence: &InclusionEvidenceRefV1,
) -> Result<(), InclusionEvidenceViolation> {
    let zero = [0_u8; 32];
    if evidence.checkpoint_digest == zero {
        return Err(InclusionEvidenceViolation::ZeroCheckpointDigest);
    }
    if evidence.tree_size == 0 {
        return Err(InclusionEvidenceViolation::EmptyTree);
    }
    if evidence.leaf_index >= evidence.tree_size {
        return Err(InclusionEvidenceViolation::LeafIndexOutOfRange);
    }
    if evidence.leaf_digest == zero {
        return Err(InclusionEvidenceViolation::ZeroLeafDigest);
    }
    if evidence.inclusion_proof_digest == zero {
        return Err(InclusionEvidenceViolation::ZeroInclusionProofDigest);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WitnessAttestationV1 {
    pub witness_profile_id: String,
    pub checkpoint_digest: Digest32,
    pub witness_key_digest: Digest32,
    pub control_domain_digest: Digest32,
    pub control_domain_credential_digest: Digest32,
    pub observation_evidence_digest: Digest32,
    pub attestation_digest: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WitnessQuorumPolicyV1 {
    pub witness_profile_id: String,
    pub required_distinct_control_domains: u16,
    pub minimum_total_witnesses: u16,
    pub control_domain_profile_digest: Digest32,
    pub witness_signature_profile_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WitnessQuorumViolation {
    WrongWitnessProfile,
    ZeroRequiredControlDomains,
    ZeroMinimumWitnesses,
    MinimumWitnessesBelowControlDomains,
    ZeroControlDomainProfileDigest,
    ZeroWitnessSignatureProfileDigest,
    NoAttestations,
    AttestationProfileMismatch,
    CheckpointMismatch,
    ZeroWitnessKeyDigest,
    ZeroControlDomainDigest,
    ZeroControlDomainCredentialDigest,
    ZeroObservationEvidenceDigest,
    ZeroAttestationDigest,
    DuplicateWitnessKey,
    InsufficientTotalWitnesses,
    InsufficientIndependentControlDomains,
}

pub fn validate_witness_quorum(
    checkpoint_digest: Digest32,
    policy: &WitnessQuorumPolicyV1,
    attestations: &[WitnessAttestationV1],
) -> Result<(), WitnessQuorumViolation> {
    let zero = [0_u8; 32];
    if policy.witness_profile_id != WITNESS_PROFILE_ID {
        return Err(WitnessQuorumViolation::WrongWitnessProfile);
    }
    if policy.required_distinct_control_domains == 0 {
        return Err(WitnessQuorumViolation::ZeroRequiredControlDomains);
    }
    if policy.minimum_total_witnesses == 0 {
        return Err(WitnessQuorumViolation::ZeroMinimumWitnesses);
    }
    if policy.minimum_total_witnesses < policy.required_distinct_control_domains {
        return Err(WitnessQuorumViolation::MinimumWitnessesBelowControlDomains);
    }
    if policy.control_domain_profile_digest == zero {
        return Err(WitnessQuorumViolation::ZeroControlDomainProfileDigest);
    }
    if policy.witness_signature_profile_digest == zero {
        return Err(WitnessQuorumViolation::ZeroWitnessSignatureProfileDigest);
    }
    if attestations.is_empty() {
        return Err(WitnessQuorumViolation::NoAttestations);
    }

    let mut witness_keys = BTreeSet::new();
    let mut control_domains = BTreeSet::new();

    for attestation in attestations {
        if attestation.witness_profile_id != WITNESS_PROFILE_ID {
            return Err(WitnessQuorumViolation::AttestationProfileMismatch);
        }
        if attestation.checkpoint_digest != checkpoint_digest {
            return Err(WitnessQuorumViolation::CheckpointMismatch);
        }
        if attestation.witness_key_digest == zero {
            return Err(WitnessQuorumViolation::ZeroWitnessKeyDigest);
        }
        if attestation.control_domain_digest == zero {
            return Err(WitnessQuorumViolation::ZeroControlDomainDigest);
        }
        if attestation.control_domain_credential_digest == zero {
            return Err(WitnessQuorumViolation::ZeroControlDomainCredentialDigest);
        }
        if attestation.observation_evidence_digest == zero {
            return Err(WitnessQuorumViolation::ZeroObservationEvidenceDigest);
        }
        if attestation.attestation_digest == zero {
            return Err(WitnessQuorumViolation::ZeroAttestationDigest);
        }
        if !witness_keys.insert(attestation.witness_key_digest) {
            return Err(WitnessQuorumViolation::DuplicateWitnessKey);
        }
        control_domains.insert(attestation.control_domain_digest);
    }

    if witness_keys.len() < usize::from(policy.minimum_total_witnesses) {
        return Err(WitnessQuorumViolation::InsufficientTotalWitnesses);
    }
    if control_domains.len() < usize::from(policy.required_distinct_control_domains) {
        return Err(WitnessQuorumViolation::InsufficientIndependentControlDomains);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WitnessEquivocationEvidenceV1 {
    pub witness_key_digest: Digest32,
    pub control_domain_digest: Digest32,
    pub checkpoint_sequence: u64,
    pub first_checkpoint_digest: Digest32,
    pub second_checkpoint_digest: Digest32,
    pub evidence_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WitnessEquivocationViolation {
    ZeroWitnessKeyDigest,
    ZeroControlDomainDigest,
    ZeroFirstCheckpointDigest,
    ZeroSecondCheckpointDigest,
    SameCheckpoint,
    ZeroEvidenceDigest,
}

pub fn validate_witness_equivocation_evidence(
    evidence: &WitnessEquivocationEvidenceV1,
) -> Result<(), WitnessEquivocationViolation> {
    let zero = [0_u8; 32];
    if evidence.witness_key_digest == zero {
        return Err(WitnessEquivocationViolation::ZeroWitnessKeyDigest);
    }
    if evidence.control_domain_digest == zero {
        return Err(WitnessEquivocationViolation::ZeroControlDomainDigest);
    }
    if evidence.first_checkpoint_digest == zero {
        return Err(WitnessEquivocationViolation::ZeroFirstCheckpointDigest);
    }
    if evidence.second_checkpoint_digest == zero {
        return Err(WitnessEquivocationViolation::ZeroSecondCheckpointDigest);
    }
    if evidence.first_checkpoint_digest == evidence.second_checkpoint_digest {
        return Err(WitnessEquivocationViolation::SameCheckpoint);
    }
    if evidence.evidence_digest == zero {
        return Err(WitnessEquivocationViolation::ZeroEvidenceDigest);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn genesis() -> ElectionTransparencyCheckpointV1 {
        ElectionTransparencyCheckpointV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.into(),
            transparency_profile_id: TRANSPARENCY_PROFILE_ID.into(),
            election_constitution_digest: digest(1),
            log_parameters_digest: digest(2),
            canonicalization_profile_digest: digest(3),
            checkpoint_sequence: 0,
            tree_size: 0,
            root_digest: digest(4),
            previous_checkpoint_digest: None,
            consistency_proof_digest: None,
        }
    }

    fn successor() -> ElectionTransparencyCheckpointV1 {
        ElectionTransparencyCheckpointV1 {
            checkpoint_sequence: 1,
            tree_size: 7,
            root_digest: digest(5),
            previous_checkpoint_digest: Some(digest(9)),
            consistency_proof_digest: Some(digest(6)),
            ..genesis()
        }
    }

    fn policy() -> WitnessQuorumPolicyV1 {
        WitnessQuorumPolicyV1 {
            witness_profile_id: WITNESS_PROFILE_ID.into(),
            required_distinct_control_domains: 3,
            minimum_total_witnesses: 3,
            control_domain_profile_digest: digest(20),
            witness_signature_profile_digest: digest(21),
        }
    }

    fn attestation(key: u8, domain: u8) -> WitnessAttestationV1 {
        WitnessAttestationV1 {
            witness_profile_id: WITNESS_PROFILE_ID.into(),
            checkpoint_digest: digest(30),
            witness_key_digest: digest(key),
            control_domain_digest: digest(domain),
            control_domain_credential_digest: digest(domain + 20),
            observation_evidence_digest: digest(key + 20),
            attestation_digest: digest(key + 40),
        }
    }

    #[test]
    fn successor_requires_exact_predecessor_and_growth() {
        let previous = genesis();
        let next = successor();
        assert_eq!(
            validate_checkpoint_successor(&previous, digest(9), &next),
            Ok(())
        );
    }

    #[test]
    fn successor_rejects_same_tree_size() {
        let previous = genesis();
        let mut next = successor();
        next.tree_size = previous.tree_size;
        assert_eq!(
            validate_checkpoint_successor(&previous, digest(9), &next),
            Err(CheckpointSuccessorViolation::TreeDidNotGrow)
        );
    }

    #[test]
    fn same_sequence_with_same_parent_and_different_root_is_equivocation() {
        let first = CheckpointRefV1 {
            checkpoint_digest: digest(40),
            election_constitution_digest: digest(1),
            checkpoint_sequence: 4,
            tree_size: 100,
            root_digest: digest(41),
            previous_checkpoint_digest: Some(digest(39)),
        };
        let mut second = first.clone();
        second.checkpoint_digest = digest(42);
        second.root_digest = digest(43);
        assert_eq!(
            classify_checkpoint_pair(&first, &second),
            CheckpointPairClassification::EquivocatingSuccessors
        );
    }

    #[test]
    fn inclusion_ref_rejects_index_outside_tree() {
        let evidence = InclusionEvidenceRefV1 {
            checkpoint_digest: digest(30),
            tree_size: 10,
            leaf_index: 10,
            leaf_digest: digest(31),
            inclusion_proof_digest: digest(32),
        };
        assert_eq!(
            validate_inclusion_evidence_ref(&evidence),
            Err(InclusionEvidenceViolation::LeafIndexOutOfRange)
        );
    }

    #[test]
    fn three_keys_under_one_controller_do_not_satisfy_independence() {
        let attestations = vec![
            attestation(1, 8),
            attestation(2, 8),
            attestation(3, 8),
        ];
        assert_eq!(
            validate_witness_quorum(digest(30), &policy(), &attestations),
            Err(WitnessQuorumViolation::InsufficientIndependentControlDomains)
        );
    }

    #[test]
    fn independent_control_domains_can_satisfy_quorum() {
        let attestations = vec![
            attestation(1, 8),
            attestation(2, 9),
            attestation(3, 10),
        ];
        assert_eq!(validate_witness_quorum(digest(30), &policy(), &attestations), Ok(()));
    }

    #[test]
    fn duplicate_witness_key_cannot_be_counted_twice() {
        let attestations = vec![
            attestation(1, 8),
            attestation(1, 9),
            attestation(3, 10),
        ];
        assert_eq!(
            validate_witness_quorum(digest(30), &policy(), &attestations),
            Err(WitnessQuorumViolation::DuplicateWitnessKey)
        );
    }

    #[test]
    fn witness_equivocation_requires_two_distinct_checkpoints() {
        let evidence = WitnessEquivocationEvidenceV1 {
            witness_key_digest: digest(1),
            control_domain_digest: digest(2),
            checkpoint_sequence: 4,
            first_checkpoint_digest: digest(3),
            second_checkpoint_digest: digest(3),
            evidence_digest: digest(4),
        };
        assert_eq!(
            validate_witness_equivocation_evidence(&evidence),
            Err(WitnessEquivocationViolation::SameCheckpoint)
        );
    }
}
