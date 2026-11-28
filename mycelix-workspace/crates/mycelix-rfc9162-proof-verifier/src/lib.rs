//! RFC 9162 SHA-256 Merkle Tree Hash primitives and proof verification.
//!
//! These functions verify proof paths relative to caller-supplied roots. They do
//! not authenticate a log identity, tree-head signer, witness quorum, split view,
//! source-universe completeness, payload truth, or authorization.
//!
//! Algorithms follow RFC 9162 sections 2.1.1, 2.1.3.2 and 2.1.4.2.

use sha2::{Digest, Sha256};
use std::fmt;

pub type Hash = [u8; 32];
pub const MAX_INCLUSION_PATH_NODES: usize = 64;
pub const MAX_CONSISTENCY_PATH_NODES: usize = 65;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InclusionDisposition {
    InclusionValidRelativeToSuppliedRoot,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConsistencyDisposition {
    ConsistencyValidRelativeToSuppliedRoots,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProofError {
    EmptyTree,
    LeafIndexOutOfRange,
    InvalidTreeSizeOrder,
    EmptyConsistencyPath,
    ProofPathTooLong,
    MalformedInclusionPath,
    MalformedConsistencyPath,
    RootMismatch,
}

impl fmt::Display for ProofError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyTree => "tree size must be non-zero",
            Self::LeafIndexOutOfRange => "leaf index is outside the supplied tree",
            Self::InvalidTreeSizeOrder => "consistency proof requires 0 < old_size < new_size",
            Self::EmptyConsistencyPath => "RFC 9162 consistency path must not be empty",
            Self::ProofPathTooLong => "proof path exceeds the RFC 9162 u64 tree-size bound",
            Self::MalformedInclusionPath => "inclusion path has missing or extra nodes",
            Self::MalformedConsistencyPath => "consistency path has missing or extra nodes",
            Self::RootMismatch => "proof does not recompute to the supplied root hash",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ProofError {}

/// SHA-256(0x00 || entry), the RFC 9162 Merkle leaf hash.
pub fn leaf_hash(entry: &[u8]) -> Hash {
    let mut hasher = Sha256::new();
    hasher.update([0x00]);
    hasher.update(entry);
    hasher.finalize().into()
}

/// SHA-256(0x01 || left || right), the RFC 9162 Merkle node hash.
pub fn node_hash(left: &Hash, right: &Hash) -> Hash {
    let mut hasher = Sha256::new();
    hasher.update([0x01]);
    hasher.update(left);
    hasher.update(right);
    hasher.finalize().into()
}

pub fn empty_tree_hash() -> Hash {
    Sha256::digest([]).into()
}

fn largest_power_of_two_less_than(n: usize) -> usize {
    debug_assert!(n >= 2);
    let mut k = 1usize;
    while k <= (n - 1) / 2 {
        k <<= 1;
    }
    k
}

/// MTH over already-hashed RFC 9162 leaf hashes.
pub fn merkle_tree_hash_from_leaf_hashes(leaves: &[Hash]) -> Hash {
    match leaves.len() {
        0 => empty_tree_hash(),
        1 => leaves[0],
        n => {
            let k = largest_power_of_two_less_than(n);
            let left = merkle_tree_hash_from_leaf_hashes(&leaves[..k]);
            let right = merkle_tree_hash_from_leaf_hashes(&leaves[k..]);
            node_hash(&left, &right)
        }
    }
}

/// MTH over ordered entry payloads.
pub fn merkle_tree_hash(entries: &[&[u8]]) -> Hash {
    let leaves: Vec<Hash> = entries.iter().map(|entry| leaf_hash(entry)).collect();
    merkle_tree_hash_from_leaf_hashes(&leaves)
}

/// Verify a pre-hashed leaf relative to the supplied root, index and tree size.
pub fn verify_leaf_inclusion(
    leaf: &Hash,
    leaf_index: u64,
    tree_size: u64,
    root_hash: &Hash,
    inclusion_path: &[Hash],
) -> Result<InclusionDisposition, ProofError> {
    if tree_size == 0 {
        return Err(ProofError::EmptyTree);
    }
    if leaf_index >= tree_size {
        return Err(ProofError::LeafIndexOutOfRange);
    }
    if inclusion_path.len() > MAX_INCLUSION_PATH_NODES {
        return Err(ProofError::ProofPathTooLong);
    }

    let mut fn_index = leaf_index;
    let mut sn_index = tree_size - 1;
    let mut calculated_root = *leaf;

    for proof_node in inclusion_path {
        if sn_index == 0 {
            return Err(ProofError::MalformedInclusionPath);
        }
        if (fn_index & 1) == 1 || fn_index == sn_index {
            calculated_root = node_hash(proof_node, &calculated_root);
            if (fn_index & 1) == 0 {
                while (fn_index & 1) == 0 && fn_index != 0 {
                    fn_index >>= 1;
                    sn_index >>= 1;
                }
            }
        } else {
            calculated_root = node_hash(&calculated_root, proof_node);
        }
        fn_index >>= 1;
        sn_index >>= 1;
    }

    if sn_index != 0 {
        return Err(ProofError::MalformedInclusionPath);
    }
    if &calculated_root != root_hash {
        return Err(ProofError::RootMismatch);
    }
    Ok(InclusionDisposition::InclusionValidRelativeToSuppliedRoot)
}

pub fn verify_entry_inclusion(
    entry: &[u8],
    leaf_index: u64,
    tree_size: u64,
    root_hash: &Hash,
    inclusion_path: &[Hash],
) -> Result<InclusionDisposition, ProofError> {
    verify_leaf_inclusion(
        &leaf_hash(entry),
        leaf_index,
        tree_size,
        root_hash,
        inclusion_path,
    )
}

/// Verify the append-only relation between two caller-supplied tree roots.
///
/// RFC 9162 requires 0 < old_size < new_size. The function does not authenticate
/// either tree head, their log identity, or any witness / split-view policy.
pub fn verify_consistency(
    old_size: u64,
    old_root: &Hash,
    new_size: u64,
    new_root: &Hash,
    consistency_path: &[Hash],
) -> Result<ConsistencyDisposition, ProofError> {
    if old_size == 0 || old_size >= new_size {
        return Err(ProofError::InvalidTreeSizeOrder);
    }
    if consistency_path.is_empty() {
        return Err(ProofError::EmptyConsistencyPath);
    }
    if consistency_path.len() > MAX_CONSISTENCY_PATH_NODES {
        return Err(ProofError::ProofPathTooLong);
    }

    let mut path = Vec::with_capacity(consistency_path.len() + 1);
    if old_size.is_power_of_two() {
        path.push(*old_root);
    }
    path.extend_from_slice(consistency_path);
    if path.len() > MAX_CONSISTENCY_PATH_NODES {
        return Err(ProofError::ProofPathTooLong);
    }

    let mut fn_index = old_size - 1;
    let mut sn_index = new_size - 1;
    if (fn_index & 1) == 1 {
        while (fn_index & 1) == 1 {
            fn_index >>= 1;
            sn_index >>= 1;
        }
    }

    let mut nodes = path.iter();
    let first = *nodes.next().ok_or(ProofError::EmptyConsistencyPath)?;
    let mut first_root = first;
    let mut second_root = first;

    for proof_node in nodes {
        if sn_index == 0 {
            return Err(ProofError::MalformedConsistencyPath);
        }
        if (fn_index & 1) == 1 || fn_index == sn_index {
            first_root = node_hash(proof_node, &first_root);
            second_root = node_hash(proof_node, &second_root);
            if (fn_index & 1) == 0 {
                while (fn_index & 1) == 0 && fn_index != 0 {
                    fn_index >>= 1;
                    sn_index >>= 1;
                }
            }
        } else {
            second_root = node_hash(&second_root, proof_node);
        }
        fn_index >>= 1;
        sn_index >>= 1;
    }

    if sn_index != 0 {
        return Err(ProofError::MalformedConsistencyPath);
    }
    if &first_root != old_root || &second_root != new_root {
        return Err(ProofError::RootMismatch);
    }
    Ok(ConsistencyDisposition::ConsistencyValidRelativeToSuppliedRoots)
}

#[cfg(test)]
mod tests;
