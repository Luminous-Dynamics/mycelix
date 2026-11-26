#![forbid(unsafe_code)]

//! RFC 9162 Merkle Tree Hash, inclusion-proof, and consistency-proof support.
//!
//! Hashing is SHA-256 with RFC 9162 domain separation:
//! - leaf: SHA256(0x00 || entry)
//! - node: SHA256(0x01 || left || right)
//!
//! This crate verifies proof bytes against supplied roots. It does not
//! authenticate who signed a tree head, whether a log is trusted, whether
//! different clients received the same tree head, or whether the leaf payload
//! satisfies application policy. Those checks belong to the caller's trust
//! and witness layer.

mod sparse_map;

pub use sparse_map::{
    SPARSE_MERKLE_DEPTH_V1, SparseKeyV1, SparseMerkleErrorV1, SparseMerkleProofV1,
    SparseMerkleTreeV1, SparseValueV1, assertion_index_key_v1, assertion_index_value_v1,
    verify_sparse_membership_v1, verify_sparse_non_membership_v1,
};

use sha2::{Digest, Sha256};

pub type Hash32 = [u8; 32];

const LEAF_PREFIX: u8 = 0x00;
const NODE_PREFIX: u8 = 0x01;
const MAX_PROOF_NODES: usize = 65;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MerkleProofErrorV1 {
    InvalidTreeSize,
    InvalidLeafIndex,
    InvalidConsistencyRange,
    EmptyConsistencyProof,
    TooManyProofNodes,
    MissingProofNodes,
    ExtraProofNodes,
    RootMismatch,
    InvalidEntrySet,
}

/// SHA-256 of arbitrary bytes.
pub fn sha256(bytes: &[u8]) -> Hash32 {
    Sha256::digest(bytes).into()
}

/// RFC 9162 Merkle leaf hash: SHA256(0x00 || entry).
pub fn leaf_hash(entry: &[u8]) -> Hash32 {
    let mut hasher = Sha256::new();
    hasher.update([LEAF_PREFIX]);
    hasher.update(entry);
    hasher.finalize().into()
}

/// RFC 9162 Merkle interior-node hash: SHA256(0x01 || left || right).
pub fn node_hash(left: &Hash32, right: &Hash32) -> Hash32 {
    let mut hasher = Sha256::new();
    hasher.update([NODE_PREFIX]);
    hasher.update(left);
    hasher.update(right);
    hasher.finalize().into()
}

/// RFC 9162 empty-tree hash: SHA256(empty).
pub fn empty_tree_hash() -> Hash32 {
    sha256(&[])
}

/// Compute the RFC 9162 Merkle Tree Hash over ordered log-entry payloads.
pub fn tree_hash(entries: &[Vec<u8>]) -> Result<Hash32, MerkleProofErrorV1> {
    if entries.is_empty() {
        return Ok(empty_tree_hash());
    }
    let leaves: Vec<Hash32> = entries.iter().map(|entry| leaf_hash(entry)).collect();
    tree_hash_from_leaf_hashes(&leaves)
}

/// Compute a Merkle Tree Hash over already domain-separated leaf hashes.
pub fn tree_hash_from_leaf_hashes(leaves: &[Hash32]) -> Result<Hash32, MerkleProofErrorV1> {
    if leaves.is_empty() {
        return Ok(empty_tree_hash());
    }
    Ok(tree_hash_range(leaves))
}

fn tree_hash_range(leaves: &[Hash32]) -> Hash32 {
    match leaves.len() {
        0 => empty_tree_hash(),
        1 => leaves[0],
        count => {
            let split = largest_power_of_two_less_than(count);
            let left = tree_hash_range(&leaves[..split]);
            let right = tree_hash_range(&leaves[split..]);
            node_hash(&left, &right)
        }
    }
}

fn largest_power_of_two_less_than(value: usize) -> usize {
    debug_assert!(value > 1);
    1_usize << (usize::BITS - 1 - (value - 1).leading_zeros())
}

/// Generate the RFC 9162 inclusion path for an entry at a zero-based index.
/// This helper is useful for log producers, conformance tests, and fixtures.
pub fn inclusion_proof(
    entries: &[Vec<u8>],
    leaf_index: u64,
) -> Result<Vec<Hash32>, MerkleProofErrorV1> {
    if entries.is_empty() {
        return Err(MerkleProofErrorV1::InvalidTreeSize);
    }
    if leaf_index >= entries.len() as u64 {
        return Err(MerkleProofErrorV1::InvalidLeafIndex);
    }
    let leaves: Vec<Hash32> = entries.iter().map(|entry| leaf_hash(entry)).collect();
    let mut proof = Vec::new();
    inclusion_path(&leaves, leaf_index as usize, &mut proof);
    Ok(proof)
}

fn inclusion_path(leaves: &[Hash32], index: usize, proof: &mut Vec<Hash32>) {
    if leaves.len() <= 1 {
        return;
    }
    let split = largest_power_of_two_less_than(leaves.len());
    if index < split {
        inclusion_path(&leaves[..split], index, proof);
        proof.push(tree_hash_range(&leaves[split..]));
    } else {
        inclusion_path(&leaves[split..], index - split, proof);
        proof.push(tree_hash_range(&leaves[..split]));
    }
}

/// Verify an RFC 9162 inclusion path for a precomputed RFC leaf hash.
pub fn verify_inclusion_proof(
    leaf: &Hash32,
    leaf_index: u64,
    tree_size: u64,
    inclusion_path: &[Hash32],
    root_hash: &Hash32,
) -> Result<(), MerkleProofErrorV1> {
    if tree_size == 0 {
        return Err(MerkleProofErrorV1::InvalidTreeSize);
    }
    if leaf_index >= tree_size {
        return Err(MerkleProofErrorV1::InvalidLeafIndex);
    }
    if inclusion_path.len() > MAX_PROOF_NODES {
        return Err(MerkleProofErrorV1::TooManyProofNodes);
    }

    let mut fn_index = leaf_index;
    let mut sn_index = tree_size - 1;
    let mut calculated_root = *leaf;

    for sibling in inclusion_path {
        if sn_index == 0 {
            return Err(MerkleProofErrorV1::ExtraProofNodes);
        }
        if (fn_index & 1) == 1 || fn_index == sn_index {
            calculated_root = node_hash(sibling, &calculated_root);
            if (fn_index & 1) == 0 {
                while fn_index != 0 && (fn_index & 1) == 0 {
                    fn_index >>= 1;
                    sn_index >>= 1;
                }
            }
        } else {
            calculated_root = node_hash(&calculated_root, sibling);
        }
        fn_index >>= 1;
        sn_index >>= 1;
    }

    if sn_index != 0 {
        return Err(MerkleProofErrorV1::MissingProofNodes);
    }
    if &calculated_root != root_hash {
        return Err(MerkleProofErrorV1::RootMismatch);
    }
    Ok(())
}

/// Verify an RFC 9162 inclusion path directly from the original entry bytes.
pub fn verify_entry_inclusion_proof(
    entry: &[u8],
    leaf_index: u64,
    tree_size: u64,
    inclusion_path: &[Hash32],
    root_hash: &Hash32,
) -> Result<(), MerkleProofErrorV1> {
    verify_inclusion_proof(
        &leaf_hash(entry),
        leaf_index,
        tree_size,
        inclusion_path,
        root_hash,
    )
}

/// Generate the RFC 9162 minimal consistency proof from the first tree size
/// to the full entry list. A first size of zero is represented by an empty
/// proof; equal sizes also require no extension proof.
pub fn consistency_proof(
    entries: &[Vec<u8>],
    first_tree_size: u64,
) -> Result<Vec<Hash32>, MerkleProofErrorV1> {
    let tree_size = entries.len() as u64;
    if first_tree_size > tree_size {
        return Err(MerkleProofErrorV1::InvalidConsistencyRange);
    }
    if first_tree_size == 0 || first_tree_size == tree_size {
        return Ok(Vec::new());
    }
    let leaves: Vec<Hash32> = entries.iter().map(|entry| leaf_hash(entry)).collect();
    let mut proof = Vec::new();
    consistency_subproof(first_tree_size as usize, &leaves, true, &mut proof);
    Ok(proof)
}

fn consistency_subproof(
    first_size: usize,
    leaves: &[Hash32],
    complete_subtree_known: bool,
    proof: &mut Vec<Hash32>,
) {
    if first_size == leaves.len() {
        if !complete_subtree_known {
            proof.push(tree_hash_range(leaves));
        }
        return;
    }

    let split = largest_power_of_two_less_than(leaves.len());
    if first_size <= split {
        consistency_subproof(first_size, &leaves[..split], complete_subtree_known, proof);
        proof.push(tree_hash_range(&leaves[split..]));
    } else {
        consistency_subproof(first_size - split, &leaves[split..], false, proof);
        proof.push(tree_hash_range(&leaves[..split]));
    }
}

/// Verify the RFC 9162 append-only consistency proof from an older tree head
/// to a newer tree head. The caller must separately authenticate both heads.
pub fn verify_consistency_proof(
    first_tree_size: u64,
    first_root: &Hash32,
    second_tree_size: u64,
    second_root: &Hash32,
    consistency_path: &[Hash32],
) -> Result<(), MerkleProofErrorV1> {
    if first_tree_size > second_tree_size {
        return Err(MerkleProofErrorV1::InvalidConsistencyRange);
    }
    if consistency_path.len() > MAX_PROOF_NODES {
        return Err(MerkleProofErrorV1::TooManyProofNodes);
    }
    if first_tree_size == 0 {
        if consistency_path.is_empty() && *first_root == empty_tree_hash() {
            return Ok(());
        }
        return Err(MerkleProofErrorV1::InvalidConsistencyRange);
    }
    if first_tree_size == second_tree_size {
        if consistency_path.is_empty() && first_root == second_root {
            return Ok(());
        }
        return Err(MerkleProofErrorV1::InvalidConsistencyRange);
    }
    if consistency_path.is_empty() {
        return Err(MerkleProofErrorV1::EmptyConsistencyProof);
    }

    let mut fn_index = first_tree_size - 1;
    let mut sn_index = second_tree_size - 1;
    while (fn_index & 1) == 1 {
        fn_index >>= 1;
        sn_index >>= 1;
    }

    let mut path_iter = consistency_path.iter();
    let first_node = path_iter
        .next()
        .ok_or(MerkleProofErrorV1::EmptyConsistencyProof)?;

    let (mut first_calculated, mut second_calculated) = if is_power_of_two(first_tree_size) {
        (*first_root, *first_root)
    } else {
        (*first_node, *first_node)
    };

    let mut nodes = path_iter;
    if is_power_of_two(first_tree_size) {
        nodes = consistency_path.iter();
    }

    for node in nodes {
        if sn_index == 0 {
            return Err(MerkleProofErrorV1::ExtraProofNodes);
        }
        if (fn_index & 1) == 1 || fn_index == sn_index {
            first_calculated = node_hash(node, &first_calculated);
            second_calculated = node_hash(node, &second_calculated);
            if (fn_index & 1) == 0 {
                while fn_index != 0 && (fn_index & 1) == 0 {
                    fn_index >>= 1;
                    sn_index >>= 1;
                }
            }
        } else {
            second_calculated = node_hash(&second_calculated, node);
        }
        fn_index >>= 1;
        sn_index >>= 1;
    }

    if sn_index != 0 {
        return Err(MerkleProofErrorV1::MissingProofNodes);
    }
    if &first_calculated != first_root || &second_calculated != second_root {
        return Err(MerkleProofErrorV1::RootMismatch);
    }
    Ok(())
}

fn is_power_of_two(value: u64) -> bool {
    value != 0 && (value & (value - 1)) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entries(count: usize) -> Vec<Vec<u8>> {
        (0..count)
            .map(|index| format!("entry-{index}").into_bytes())
            .collect()
    }

    #[test]
    fn sha256_known_answer_for_empty_input() {
        assert_eq!(
            hex(&sha256(b"")),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
    }

    #[test]
    fn leaf_and_node_hashes_are_domain_separated() {
        let payload = b"same bytes";
        assert_ne!(leaf_hash(payload), sha256(payload));
        assert_ne!(
            node_hash(&leaf_hash(payload), &leaf_hash(payload)),
            leaf_hash(payload)
        );
    }

    #[test]
    fn exhaustive_inclusion_proofs_for_tree_sizes_one_through_sixty_four() {
        for size in 1..=64 {
            let records = entries(size);
            let root = tree_hash(&records).unwrap();
            for index in 0..size {
                let proof = inclusion_proof(&records, index as u64).unwrap();
                verify_entry_inclusion_proof(
                    &records[index],
                    index as u64,
                    size as u64,
                    &proof,
                    &root,
                )
                .unwrap();
            }
        }
    }

    #[test]
    fn inclusion_rejects_wrong_index_leaf_root_and_path_shape() {
        let records = entries(7);
        let root = tree_hash(&records).unwrap();
        let proof = inclusion_proof(&records, 4).unwrap();

        assert_eq!(
            verify_entry_inclusion_proof(&records[4], 7, 7, &proof, &root),
            Err(MerkleProofErrorV1::InvalidLeafIndex)
        );
        assert_eq!(
            verify_entry_inclusion_proof(b"wrong", 4, 7, &proof, &root),
            Err(MerkleProofErrorV1::RootMismatch)
        );
        assert_eq!(
            verify_entry_inclusion_proof(&records[4], 4, 7, &proof, &sha256(b"wrong-root")),
            Err(MerkleProofErrorV1::RootMismatch)
        );
        let mut missing = proof.clone();
        missing.pop();
        assert_eq!(
            verify_entry_inclusion_proof(&records[4], 4, 7, &missing, &root),
            Err(MerkleProofErrorV1::MissingProofNodes)
        );
        let mut extra = proof;
        extra.push(sha256(b"extra"));
        assert_eq!(
            verify_entry_inclusion_proof(&records[4], 4, 7, &extra, &root),
            Err(MerkleProofErrorV1::ExtraProofNodes)
        );
    }

    #[test]
    fn exhaustive_consistency_proofs_for_all_prefix_pairs_through_sixty_four() {
        for size in 1..=64 {
            let records = entries(size);
            let new_root = tree_hash(&records).unwrap();
            for old_size in 0..=size {
                let old_root = tree_hash(&records[..old_size]).unwrap();
                let proof = consistency_proof(&records, old_size as u64).unwrap();
                verify_consistency_proof(
                    old_size as u64,
                    &old_root,
                    size as u64,
                    &new_root,
                    &proof,
                )
                .unwrap_or_else(|error| panic!("sizes {old_size}->{size}: {error:?}"));
            }
        }
    }

    #[test]
    fn consistency_rejects_truncation_fork_and_wrong_sizes() {
        let records = entries(17);
        let old_root = tree_hash(&records[..9]).unwrap();
        let new_root = tree_hash(&records).unwrap();
        let proof = consistency_proof(&records, 9).unwrap();

        assert_eq!(
            verify_consistency_proof(10, &old_root, 9, &new_root, &proof),
            Err(MerkleProofErrorV1::InvalidConsistencyRange)
        );
        assert_eq!(
            verify_consistency_proof(9, &sha256(b"wrong-old"), 17, &new_root, &proof),
            Err(MerkleProofErrorV1::RootMismatch)
        );
        assert_eq!(
            verify_consistency_proof(9, &old_root, 17, &sha256(b"wrong-new"), &proof),
            Err(MerkleProofErrorV1::RootMismatch)
        );
        let mut missing = proof.clone();
        missing.pop();
        assert!(verify_consistency_proof(9, &old_root, 17, &new_root, &missing).is_err());

        assert_eq!(
            verify_consistency_proof(9, &old_root, 9, &new_root, &proof),
            Err(MerkleProofErrorV1::InvalidConsistencyRange)
        );
    }

    #[test]
    fn fixed_seven_entry_vector_matches_independent_sha256_fixture() {
        let records: Vec<Vec<u8>> = (b'a'..=b'g').map(|byte| vec![byte]).collect();
        let root = tree_hash(&records).unwrap();
        assert_eq!(
            hex(&root),
            "4ae191939f548d9934740b88dea2c5cb89bb8870fc4505cd79dec6bbfaaee9cb"
        );

        let inclusion = inclusion_proof(&records, 6).unwrap();
        assert_eq!(
            inclusion.iter().map(|hash| hex(hash)).collect::<Vec<_>>(),
            vec![
                "918566184c9d5be235ad2b6dd60828f5cec14fc409f02f7db8647009ec6da588",
                "33376a3bd63e9993708a84ddfe6c28ae58b83505dd1fed711bd924ec5a6239f0",
            ]
        );
        verify_entry_inclusion_proof(&records[6], 6, 7, &inclusion, &root).unwrap();

        let old_root = tree_hash(&records[..3]).unwrap();
        assert_eq!(
            hex(&old_root),
            "36642e73c2540ab121e3a6bf9545b0a24982cd830eb13d3cd19de3ce6c021ec1"
        );
        let consistency = consistency_proof(&records, 3).unwrap();
        assert_eq!(
            consistency.iter().map(|hash| hex(hash)).collect::<Vec<_>>(),
            vec![
                "597fcb31282d34654c200d3418fca5705c648ebf326ec73d8ddef11841f876d8",
                "d070dc5b8da9aea7dc0f5ad4c29d89965200059c9a0ceca3abd5da2492dcb71d",
                "b137985ff484fb600db93107c77b0365c80d78f5b429ded0fd97361d077999eb",
                "e286d3390665a7cdc759453bed0b00cded1842d757e3e6cfe87df53db177e725",
            ]
        );
        verify_consistency_proof(3, &old_root, 7, &root, &consistency).unwrap();
    }

    #[test]
    fn inclusion_and_consistency_paths_match_rfc_9162_seven_leaf_example_shape() {
        // RFC 9162 Section 2.1.5 names the seven leaves d0..d6 and states:
        // PATH(6) = [i, k], with i=MTH(d4,d5), k=MTH(d0..d3).
        let records: Vec<Vec<u8>> = (0_u8..7).map(|index| vec![index]).collect();
        let path_d6 = inclusion_proof(&records, 6).unwrap();
        assert_eq!(
            path_d6,
            vec![
                tree_hash(&records[4..6]).unwrap(),
                tree_hash(&records[..4]).unwrap(),
            ]
        );

        // The RFC also gives PROOF(3,D7) = [c,d,g,l].
        let proof_3_to_7 = consistency_proof(&records, 3).unwrap();
        assert_eq!(
            proof_3_to_7,
            vec![
                tree_hash(&records[2..3]).unwrap(),
                tree_hash(&records[3..4]).unwrap(),
                tree_hash(&records[..2]).unwrap(),
                tree_hash(&records[4..7]).unwrap(),
            ]
        );
        verify_consistency_proof(
            3,
            &tree_hash(&records[..3]).unwrap(),
            7,
            &tree_hash(&records).unwrap(),
            &proof_3_to_7,
        )
        .unwrap();
    }

    #[test]
    fn zero_size_and_equal_size_consistency_edges_are_explicit() {
        let one = entries(1);
        let one_root = tree_hash(&one).unwrap();
        assert!(verify_consistency_proof(0, &empty_tree_hash(), 1, &one_root, &[]).is_ok());
        assert!(verify_consistency_proof(1, &one_root, 1, &one_root, &[]).is_ok());
        assert!(verify_consistency_proof(0, &sha256(b"not-empty"), 1, &one_root, &[]).is_err());
    }

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|byte| format!("{byte:02x}")).collect()
    }
}
