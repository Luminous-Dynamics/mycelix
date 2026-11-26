#![forbid(unsafe_code)]

//! Experimental 256-bit sparse Merkle map for bounded key membership and
//! non-membership proofs. This is a separate VDS profile from RFC 9162's
//! append-only Merkle Tree Hash. It is not currently an RFC 9942/IANA VDS
//! registration and must not be labelled RFC 9162_SHA256.

use std::collections::BTreeMap;

use sha2::{Digest, Sha256};

pub type SparseKeyV1 = [u8; 32];
pub type SparseValueV1 = [u8; 32];

pub const SPARSE_MERKLE_DEPTH_V1: usize = 256;
const SPARSE_DOMAIN: &[u8] = b"MYCELIX-EXPERIMENTAL-SMT-SHA256-V1\0";
const KEY_DOMAIN: &[u8] = b"MYCELIX-ASSERTION-INDEX-KEY-V1\0";
const VALUE_DOMAIN: &[u8] = b"MYCELIX-ASSERTION-INDEX-VALUE-V1\0";

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SparseMerkleProofV1 {
    /// Siblings ordered from leaf level to root level (key bit 255 to bit 0).
    pub siblings_leaf_to_root: Vec<SparseValueV1>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SparseMerkleErrorV1 {
    DuplicateKey,
    InvalidProofLength,
    RootMismatch,
    EmptyIdentifier,
    IdentifierTooLong,
    TextTooLong,
    InvalidOwnerSequence,
    InvalidDigest,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct SparseMerkleTreeV1 {
    entries: BTreeMap<SparseKeyV1, SparseValueV1>,
}

impl SparseMerkleTreeV1 {
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert one key/value pair. Duplicate keys are rejected rather than
    /// silently selecting a winner; callers that model updates must create a
    /// new map state/checkpoint or use an explicit replacement operation.
    pub fn insert(
        &mut self,
        key: SparseKeyV1,
        value: SparseValueV1,
    ) -> Result<(), SparseMerkleErrorV1> {
        if self.entries.contains_key(&key) {
            return Err(SparseMerkleErrorV1::DuplicateKey);
        }
        self.entries.insert(key, value);
        Ok(())
    }

    /// Explicitly replace an existing mapping. The previous value is returned.
    pub fn replace(&mut self, key: SparseKeyV1, value: SparseValueV1) -> Option<SparseValueV1> {
        self.entries
            .get_mut(&key)
            .map(|existing| std::mem::replace(existing, value))
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn root_hash(&self) -> SparseValueV1 {
        let defaults = default_hashes();
        let entries: Vec<(SparseKeyV1, SparseValueV1)> = self
            .entries
            .iter()
            .map(|(key, value)| (*key, *value))
            .collect();
        subtree_hash(&entries, 0, &defaults)
    }

    /// Produce a proof for either a present or absent key.
    pub fn prove(&self, key: SparseKeyV1) -> SparseMerkleProofV1 {
        let defaults = default_hashes();
        let entries: Vec<(SparseKeyV1, SparseValueV1)> = self
            .entries
            .iter()
            .map(|(entry_key, value)| (*entry_key, *value))
            .collect();
        let mut siblings = Vec::with_capacity(SPARSE_MERKLE_DEPTH_V1);
        build_proof(&entries, &key, 0, &defaults, &mut siblings);
        SparseMerkleProofV1 {
            siblings_leaf_to_root: siblings,
        }
    }
}

/// Deterministic query key for a verifier-scoped assertion lookup.
pub fn assertion_index_key_v1(
    verifier_ref: &str,
    policy_epoch_ref: &str,
    subject_ref: &str,
    claim_id: &str,
    dimension: &str,
    assertion_ref: &str,
) -> Result<SparseKeyV1, SparseMerkleErrorV1> {
    let mut bytes = KEY_DOMAIN.to_vec();
    append_string(&mut bytes, verifier_ref)?;
    append_string(&mut bytes, policy_epoch_ref)?;
    append_string(&mut bytes, subject_ref)?;
    append_string(&mut bytes, claim_id)?;
    append_string(&mut bytes, dimension)?;
    append_string(&mut bytes, assertion_ref)?;
    Ok(hash(&bytes))
}

/// Deterministic map value committing to the accepted receipt identity and
/// position. The receipt's canonical digest is supplied by the receipt layer.
pub fn assertion_index_value_v1(
    receipt_digest: &str,
    owner_ref: &str,
    owner_sequence: u64,
) -> Result<SparseValueV1, SparseMerkleErrorV1> {
    validate_sha256_digest(receipt_digest)?;
    validate_id(owner_ref)?;
    if owner_sequence == 0 {
        return Err(SparseMerkleErrorV1::InvalidOwnerSequence);
    }
    let mut bytes = VALUE_DOMAIN.to_vec();
    append_string(&mut bytes, receipt_digest)?;
    append_string(&mut bytes, owner_ref)?;
    bytes.extend_from_slice(&owner_sequence.to_be_bytes());
    Ok(hash(&bytes))
}

/// Verify that key -> value is present under the supplied sparse root.
pub fn verify_sparse_membership_v1(
    key: &SparseKeyV1,
    value: &SparseValueV1,
    proof: &SparseMerkleProofV1,
    root_hash: &SparseValueV1,
) -> Result<(), SparseMerkleErrorV1> {
    let current = present_leaf_hash(key, value);
    verify_path(key, current, proof, root_hash)
}

/// Verify that key is absent under the supplied sparse root.
///
/// This proves non-membership only in this exact sparse-map commitment. It
/// does not prove that a separate append-only history was completely indexed,
/// that the map owner is authoritative, or that the checkpoint is trusted.
pub fn verify_sparse_non_membership_v1(
    key: &SparseKeyV1,
    proof: &SparseMerkleProofV1,
    root_hash: &SparseValueV1,
) -> Result<(), SparseMerkleErrorV1> {
    let current = empty_leaf_hash();
    verify_path(key, current, proof, root_hash)
}

fn verify_path(
    key: &SparseKeyV1,
    mut current: SparseValueV1,
    proof: &SparseMerkleProofV1,
    root_hash: &SparseValueV1,
) -> Result<(), SparseMerkleErrorV1> {
    if proof.siblings_leaf_to_root.len() != SPARSE_MERKLE_DEPTH_V1 {
        return Err(SparseMerkleErrorV1::InvalidProofLength);
    }

    for (offset, sibling) in proof.siblings_leaf_to_root.iter().enumerate() {
        let depth = SPARSE_MERKLE_DEPTH_V1 - 1 - offset;
        current = if key_bit(key, depth) {
            branch_hash(sibling, &current)
        } else {
            branch_hash(&current, sibling)
        };
    }

    if &current != root_hash {
        return Err(SparseMerkleErrorV1::RootMismatch);
    }
    Ok(())
}

fn build_proof(
    entries: &[(SparseKeyV1, SparseValueV1)],
    key: &SparseKeyV1,
    depth: usize,
    defaults: &[SparseValueV1; SPARSE_MERKLE_DEPTH_V1 + 1],
    output: &mut Vec<SparseValueV1>,
) {
    if depth == SPARSE_MERKLE_DEPTH_V1 {
        return;
    }

    let split = entries.partition_point(|(entry_key, _)| !key_bit(entry_key, depth));
    if key_bit(key, depth) {
        let sibling = subtree_hash(&entries[..split], depth + 1, defaults);
        build_proof(&entries[split..], key, depth + 1, defaults, output);
        output.push(sibling);
    } else {
        let sibling = subtree_hash(&entries[split..], depth + 1, defaults);
        build_proof(&entries[..split], key, depth + 1, defaults, output);
        output.push(sibling);
    }
}

fn subtree_hash(
    entries: &[(SparseKeyV1, SparseValueV1)],
    depth: usize,
    defaults: &[SparseValueV1; SPARSE_MERKLE_DEPTH_V1 + 1],
) -> SparseValueV1 {
    if entries.is_empty() {
        return defaults[depth];
    }
    if depth == SPARSE_MERKLE_DEPTH_V1 {
        debug_assert_eq!(entries.len(), 1, "duplicate 256-bit keys");
        return present_leaf_hash(&entries[0].0, &entries[0].1);
    }

    let split = entries.partition_point(|(key, _)| !key_bit(key, depth));
    let left = subtree_hash(&entries[..split], depth + 1, defaults);
    let right = subtree_hash(&entries[split..], depth + 1, defaults);
    branch_hash(&left, &right)
}

fn default_hashes() -> [SparseValueV1; SPARSE_MERKLE_DEPTH_V1 + 1] {
    let mut defaults = [[0_u8; 32]; SPARSE_MERKLE_DEPTH_V1 + 1];
    defaults[SPARSE_MERKLE_DEPTH_V1] = empty_leaf_hash();
    for depth in (0..SPARSE_MERKLE_DEPTH_V1).rev() {
        defaults[depth] = branch_hash(&defaults[depth + 1], &defaults[depth + 1]);
    }
    defaults
}

fn present_leaf_hash(key: &SparseKeyV1, value: &SparseValueV1) -> SparseValueV1 {
    let mut bytes = Vec::with_capacity(1 + SPARSE_DOMAIN.len() + 64);
    bytes.push(0x10);
    bytes.extend_from_slice(SPARSE_DOMAIN);
    bytes.extend_from_slice(key);
    bytes.extend_from_slice(value);
    hash(&bytes)
}

fn empty_leaf_hash() -> SparseValueV1 {
    let mut bytes = Vec::with_capacity(1 + SPARSE_DOMAIN.len());
    bytes.push(0x12);
    bytes.extend_from_slice(SPARSE_DOMAIN);
    hash(&bytes)
}

fn branch_hash(left: &SparseValueV1, right: &SparseValueV1) -> SparseValueV1 {
    let mut bytes = [0_u8; 65];
    bytes[0] = 0x11;
    bytes[1..33].copy_from_slice(left);
    bytes[33..65].copy_from_slice(right);
    hash(&bytes)
}

fn hash(bytes: &[u8]) -> SparseValueV1 {
    Sha256::digest(bytes).into()
}

fn key_bit(key: &SparseKeyV1, depth: usize) -> bool {
    let byte = key[depth / 8];
    let shift = 7 - (depth % 8);
    ((byte >> shift) & 1) == 1
}

fn append_string(output: &mut Vec<u8>, value: &str) -> Result<(), SparseMerkleErrorV1> {
    validate_id(value)?;
    let length = u32::try_from(value.len()).map_err(|_| SparseMerkleErrorV1::IdentifierTooLong)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
    Ok(())
}

#[cfg(test)]
fn hex_lower(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

fn validate_sha256_digest(value: &str) -> Result<(), SparseMerkleErrorV1> {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return Err(SparseMerkleErrorV1::InvalidDigest);
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(SparseMerkleErrorV1::InvalidDigest);
    }
    Ok(())
}

fn validate_id(value: &str) -> Result<(), SparseMerkleErrorV1> {
    if value.trim().is_empty() {
        return Err(SparseMerkleErrorV1::EmptyIdentifier);
    }
    if value.len() > 1_024 {
        return Err(SparseMerkleErrorV1::IdentifierTooLong);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> SparseValueV1 {
        let mut value = [0_u8; 32];
        value[0] = byte;
        value
    }

    fn key(byte: u8) -> SparseKeyV1 {
        let mut key = [0_u8; 32];
        key[0] = byte;
        key
    }

    #[test]
    fn sparse_map_matches_shared_known_answer_root_and_non_membership_path() {
        let mut tree = SparseMerkleTreeV1::new();
        for (key_byte, value_byte) in [(0x00, 0xA1), (0x80, 0xA2), (0xFF, 0xA3)] {
            tree.insert(key(key_byte), digest(value_byte)).unwrap();
        }
        let root = tree.root_hash();
        assert_eq!(
            hex_lower(&root),
            "ca399e0f53e5b7607402225d68a14e607cc7e7e488e44ccf8bde194e6386718f"
        );

        let mut absent = key(0);
        absent[0] = 0x40;
        let proof = tree.prove(absent);
        assert_eq!(proof.siblings_leaf_to_root.len(), SPARSE_MERKLE_DEPTH_V1);
        assert_eq!(
            proof.siblings_leaf_to_root.len() * 32,
            8_192,
            "uncompressed 256-level proof must contain 8192 bytes"
        );
        let mut proof_bytes = Vec::with_capacity(8_192);
        for sibling in &proof.siblings_leaf_to_root {
            proof_bytes.extend_from_slice(sibling);
        }
        assert_eq!(
            hex_lower(&Sha256::digest(&proof_bytes)),
            "dc05c01529111d01b12e06f0ddb2a9b22d78822352ba8425c6768e3e3548766f"
        );
        assert_eq!(
            verify_sparse_non_membership_v1(&absent, &proof, &root),
            Ok(())
        );
    }

    #[test]
    fn sparse_empty_root_matches_shared_known_answer() {
        let tree = SparseMerkleTreeV1::new();
        assert_eq!(
            hex_lower(&tree.root_hash()),
            "de3054ab8d693834a8b74a5f07765258eb95da1c61faaf16ce437659142ee250"
        );
        let absent = key(0x81);
        assert_eq!(
            verify_sparse_non_membership_v1(&absent, &tree.prove(absent), &tree.root_hash()),
            Ok(())
        );
    }

    #[test]
    fn exhaustive_bounded_sparse_maps_verify_present_and_absent_keys() {
        for count in 0..=16_usize {
            let mut tree = SparseMerkleTreeV1::new();
            for index in 0..count {
                let mut key = [0_u8; 32];
                key[0] = (index >> 8) as u8;
                key[1] = index as u8;
                tree.insert(key, digest((index + 1) as u8)).unwrap();
            }
            let root = tree.root_hash();
            for index in 0..count {
                let mut key = [0_u8; 32];
                key[0] = (index >> 8) as u8;
                key[1] = index as u8;
                assert_eq!(
                    verify_sparse_membership_v1(
                        &key,
                        &digest((index + 1) as u8),
                        &tree.prove(key),
                        &root,
                    ),
                    Ok(()),
                    "count={count}, present={index}"
                );
            }
            for index in 128..132 {
                let mut key = [0_u8; 32];
                key[0] = (index >> 8) as u8;
                key[1] = index as u8;
                assert_eq!(
                    verify_sparse_non_membership_v1(&key, &tree.prove(key), &root),
                    Ok(()),
                    "count={count}, absent={index}"
                );
            }
        }
    }

    #[test]
    fn empty_sparse_tree_supports_non_membership() {
        let tree = SparseMerkleTreeV1::new();
        let root = tree.root_hash();
        let proof = tree.prove(key(0x42));
        assert_eq!(
            verify_sparse_non_membership_v1(&key(0x42), &proof, &root),
            Ok(())
        );
        assert_eq!(tree.len(), 0);
    }

    #[test]
    fn membership_and_non_membership_are_distinct_for_same_root() {
        let mut tree = SparseMerkleTreeV1::new();
        tree.insert(key(0x00), digest(0xA1)).unwrap();
        tree.insert(key(0x80), digest(0xB2)).unwrap();
        tree.insert(key(0xFF), digest(0xC3)).unwrap();
        let root = tree.root_hash();

        let present = tree.prove(key(0x80));
        assert_eq!(
            verify_sparse_membership_v1(&key(0x80), &digest(0xB2), &present, &root),
            Ok(())
        );
        assert_eq!(
            verify_sparse_non_membership_v1(&key(0x80), &present, &root),
            Err(SparseMerkleErrorV1::RootMismatch)
        );

        let absent = tree.prove(key(0x40));
        assert_eq!(
            verify_sparse_non_membership_v1(&key(0x40), &absent, &root),
            Ok(())
        );
        assert_eq!(
            verify_sparse_membership_v1(&key(0x40), &digest(0), &absent, &root),
            Err(SparseMerkleErrorV1::RootMismatch)
        );
    }

    #[test]
    fn proof_mutation_wrong_key_and_wrong_root_fail_closed() {
        let mut tree = SparseMerkleTreeV1::new();
        tree.insert(key(0x11), digest(0xAA)).unwrap();
        tree.insert(key(0x91), digest(0xBB)).unwrap();
        let root = tree.root_hash();
        let mut proof = tree.prove(key(0x11));

        let mut short = proof.clone();
        short.siblings_leaf_to_root.pop();
        assert_eq!(
            verify_sparse_membership_v1(&key(0x11), &digest(0xAA), &short, &root),
            Err(SparseMerkleErrorV1::InvalidProofLength)
        );

        proof.siblings_leaf_to_root[17] = digest(0xDD);
        assert_eq!(
            verify_sparse_membership_v1(&key(0x11), &digest(0xAA), &proof, &root),
            Err(SparseMerkleErrorV1::RootMismatch)
        );
        let valid = tree.prove(key(0x11));
        assert_eq!(
            verify_sparse_membership_v1(&key(0x12), &digest(0xAA), &valid, &root),
            Err(SparseMerkleErrorV1::RootMismatch)
        );
        assert_eq!(
            verify_sparse_membership_v1(&key(0x11), &digest(0xAA), &valid, &digest(0xEE)),
            Err(SparseMerkleErrorV1::RootMismatch)
        );
    }

    #[test]
    fn duplicate_keys_are_rejected_and_replace_is_explicit() {
        let mut tree = SparseMerkleTreeV1::new();
        tree.insert(key(0x01), digest(0x10)).unwrap();
        assert_eq!(
            tree.insert(key(0x01), digest(0x20)),
            Err(SparseMerkleErrorV1::DuplicateKey)
        );
        assert_eq!(tree.replace(key(0x01), digest(0x20)), Some(digest(0x10)));
        assert_eq!(tree.replace(key(0x02), digest(0x30)), None);
        assert_eq!(tree.len(), 1);
        let proof = tree.prove(key(0x01));
        assert_eq!(
            verify_sparse_membership_v1(&key(0x01), &digest(0x20), &proof, &tree.root_hash()),
            Ok(())
        );
    }

    #[test]
    fn sparse_query_key_and_value_match_go_known_answers() {
        let key = assertion_index_key_v1(
            "did:mycelix:verifier:finance-v1",
            "policy-epoch:1",
            "subject:grant:42",
            "claim:revoked",
            "VALID_AT",
            "assertion:42",
        )
        .unwrap();
        assert_eq!(
            hex_lower(&key),
            "6f02bb0fe2b2c6bd9b0c2efac3f74fdc0db676eb21ac5d5f42648428b3dc1817"
        );

        let value = assertion_index_value_v1(
            &format!("sha256:{}", "a".repeat(64)),
            "owner:verifier-log",
            7,
        )
        .unwrap();
        assert_eq!(
            hex_lower(&value),
            "d641a80447381fdce99b19eb3e9f4fc58000699bff05c81a3410cf174c5c2d7c"
        );
    }

    #[test]
    fn sparse_index_identifiers_are_bounded() {
        let too_long = "x".repeat(1_025);
        assert_eq!(
            assertion_index_key_v1(
                "verifier",
                "policy",
                &too_long,
                "claim",
                "VALID_AT",
                "assertion",
            ),
            Err(SparseMerkleErrorV1::IdentifierTooLong)
        );
    }

    #[test]
    fn query_keys_are_scoped_by_verifier_policy_subject_claim_and_dimension() {
        let baseline = assertion_index_key_v1(
            "did:mycelix:verifier:finance-v1",
            "policy-epoch:1",
            "subject:grant:42",
            "claim:revoked",
            "VALID_AT",
            "assertion:42",
        )
        .unwrap();
        assert_ne!(
            baseline,
            assertion_index_key_v1(
                "did:mycelix:verifier:other-v1",
                "policy-epoch:1",
                "subject:grant:42",
                "claim:revoked",
                "VALID_AT",
                "assertion:42",
            )
            .unwrap()
        );
        assert_ne!(
            baseline,
            assertion_index_key_v1(
                "did:mycelix:verifier:finance-v1",
                "policy-epoch:2",
                "subject:grant:42",
                "claim:revoked",
                "VALID_AT",
                "assertion:42",
            )
            .unwrap()
        );
        assert_ne!(
            baseline,
            assertion_index_key_v1(
                "did:mycelix:verifier:finance-v1",
                "policy-epoch:1",
                "subject:grant:42",
                "claim:revoked",
                "VALID_AT",
                "assertion:43",
            )
            .unwrap()
        );
    }
}
