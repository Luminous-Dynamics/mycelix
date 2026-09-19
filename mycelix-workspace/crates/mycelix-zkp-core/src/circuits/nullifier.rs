// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Nullifier utilities and a portable nullifier-proof envelope.
//!
//! This module does not itself verify zero-knowledge membership. A complete
//! backend-specific circuit must prove membership and nullifier derivation.
//! Structural envelope validation is not cryptographic verification.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

/// Stable nullifier derivation profiles.
///
/// New authority-bearing protocols should bind the exact profile instead of
/// interpreting 32 nullifier bytes without derivation provenance.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NullifierDerivationProfile {
    /// Historical delimiter-framed derivation. Ambiguous for arbitrary byte
    /// inputs and retained only for compatibility with existing values.
    LegacyDelimiterV1,
    /// Domain-separated, field-tagged, length-prefixed tuple encoding.
    LengthPrefixedV2,
}

/// Errors from fail-closed v2 nullifier derivation.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum NullifierDerivationError {
    #[error("member secret must not be empty")]
    EmptyMemberSecret,
    #[error("group id must not be empty")]
    EmptyGroupId,
    #[error("input length cannot be represented by the v2 framing")]
    InputTooLong,
}

/// A nullifier-based membership proof envelope.
///
/// This type predates explicit derivation-profile binding. It is a portable
/// envelope only; it does not prove membership, nullifier derivation, or
/// double-use prevention by itself.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NullifierProof {
    pub nullifier: [u8; 32],
    pub group_root: [u8; 32],
    pub proof_bytes: Vec<u8>,
    pub domain_tag: String,
}

/// Historical v1 nullifier derivation.
///
/// Encoding:
///
/// ```text
/// SHA256("ZTML:nullifier:v1:" || member_secret || ":" || group_id)
/// ```
///
/// Because both fields are arbitrary bytes, delimiter framing is ambiguous.
/// For example `(a, b:c)` and `(a:b, c)` hash the same preimage. New protocols
/// must use [`compute_nullifier_v2`] instead.
pub fn compute_nullifier(member_secret: &[u8], group_id: &[u8]) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(b"ZTML:nullifier:v1:");
    hasher.update(member_secret);
    hasher.update(b":");
    hasher.update(group_id);
    let result = hasher.finalize();
    let mut nullifier = [0u8; 32];
    nullifier.copy_from_slice(&result);
    nullifier
}

/// Canonical v2 nullifier derivation with unambiguous tuple framing.
///
/// Exact byte preimage:
///
/// ```text
/// "MYCELIX-NULLIFIER:v2\0"
/// "member_secret\0" || u64_le(len(member_secret)) || member_secret
/// "group_id\0"      || u64_le(len(group_id))      || group_id
/// ```
///
/// Empty member secrets and empty group identifiers are rejected. This function
/// establishes only deterministic derivation semantics; a proof system must
/// separately prove that an authenticated hidden witness opens the claimed
/// membership leaf/nullifier relation.
pub fn compute_nullifier_v2(
    member_secret: &[u8],
    group_id: &[u8],
) -> Result<[u8; 32], NullifierDerivationError> {
    if member_secret.is_empty() {
        return Err(NullifierDerivationError::EmptyMemberSecret);
    }
    if group_id.is_empty() {
        return Err(NullifierDerivationError::EmptyGroupId);
    }

    let member_len =
        u64::try_from(member_secret.len()).map_err(|_| NullifierDerivationError::InputTooLong)?;
    let group_len =
        u64::try_from(group_id.len()).map_err(|_| NullifierDerivationError::InputTooLong)?;

    let mut hasher = Sha256::new();
    hasher.update(b"MYCELIX-NULLIFIER:v2\0");
    hasher.update(b"member_secret\0");
    hasher.update(member_len.to_le_bytes());
    hasher.update(member_secret);
    hasher.update(b"group_id\0");
    hasher.update(group_len.to_le_bytes());
    hasher.update(group_id);

    let result = hasher.finalize();
    let mut nullifier = [0u8; 32];
    nullifier.copy_from_slice(&result);
    Ok(nullifier)
}

/// Derive a nullifier under an explicit profile.
pub fn compute_nullifier_with_profile(
    profile: NullifierDerivationProfile,
    member_secret: &[u8],
    group_id: &[u8],
) -> Result<[u8; 32], NullifierDerivationError> {
    match profile {
        NullifierDerivationProfile::LegacyDelimiterV1 => Ok(compute_nullifier(member_secret, group_id)),
        NullifierDerivationProfile::LengthPrefixedV2 => {
            compute_nullifier_v2(member_secret, group_id)
        }
    }
}

/// Compute a member's leaf hash for the group Merkle tree.
pub fn compute_member_leaf(member_secret: &[u8]) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(b"ZTML:member:v1:");
    hasher.update(member_secret);
    let result = hasher.finalize();
    let mut leaf = [0u8; 32];
    leaf.copy_from_slice(&result);
    leaf
}

/// Validate nullifier proof structure only.
pub fn validate_nullifier_proof_structure(proof: &NullifierProof) -> Result<(), String> {
    if proof.proof_bytes.is_empty() {
        return Err("Empty proof bytes".to_string());
    }
    if proof.nullifier == [0u8; 32] {
        return Err("Zero nullifier".to_string());
    }
    if proof.group_root == [0u8; 32] {
        return Err("Zero group root".to_string());
    }
    if !proof.domain_tag.starts_with("ZTML:") {
        return Err("Invalid domain tag".to_string());
    }
    Ok(())
}

/// Backward-compatible alias for structural validation only.
#[deprecated(
    since = "0.1.0",
    note = "structural validation only; use validate_nullifier_proof_structure"
)]
pub fn validate_nullifier_proof(proof: &NullifierProof) -> Result<(), String> {
    validate_nullifier_proof_structure(proof)
}

/// In-memory used-nullifier set.
///
/// Production protocols must use an appropriate durable/consensus-backed store;
/// this helper alone is not distributed double-use prevention.
pub struct NullifierSet {
    used: std::collections::HashSet<[u8; 32]>,
}

impl Default for NullifierSet {
    fn default() -> Self {
        Self::new()
    }
}

impl NullifierSet {
    pub fn new() -> Self {
        Self {
            used: std::collections::HashSet::new(),
        }
    }

    /// Returns true on first insertion and false if already present.
    pub fn check_and_mark(&mut self, nullifier: &[u8; 32]) -> bool {
        self.used.insert(*nullifier)
    }

    pub fn is_used(&self, nullifier: &[u8; 32]) -> bool {
        self.used.contains(nullifier)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuits::merkle_membership::{
        compute_merkle_root, generate_merkle_path, verify_merkle_path,
    };

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    #[test]
    fn v1_is_deterministic() {
        let n1 = compute_nullifier(b"alice_secret", b"care_circle_42");
        let n2 = compute_nullifier(b"alice_secret", b"care_circle_42");
        assert_eq!(n1, n2);
    }

    #[test]
    fn demonstrates_legacy_delimiter_ambiguity() {
        let one = compute_nullifier(b"a", b"b:c");
        let two = compute_nullifier(b"a:b", b"c");
        assert_eq!(one, two, "v1 compatibility behavior must remain explicit");
    }

    #[test]
    fn v2_separates_the_legacy_ambiguity_pair() {
        let one = compute_nullifier_v2(b"a", b"b:c").expect("v2 one");
        let two = compute_nullifier_v2(b"a:b", b"c").expect("v2 two");
        assert_ne!(one, two);
        assert_eq!(
            hex(&one),
            "76ed9b950567fa0a6cf0d3d0baa29385ee422c343a7f124105d996560bea137a"
        );
        assert_eq!(
            hex(&two),
            "c9f37d7caecc134105c72380883089a2520d005ec0b54680b78534c632ce95b4"
        );
    }

    #[test]
    fn v2_reference_vector_is_frozen() {
        let n = compute_nullifier_v2(b"alice_secret", b"care_circle_42").expect("v2");
        assert_eq!(
            hex(&n),
            "47a4c042a23419670f53a641cab9ee6bd1c13b5490b4f8129b9bcbbe32159bfc"
        );
    }

    #[test]
    fn v2_rejects_empty_identity_fields() {
        assert_eq!(
            compute_nullifier_v2(b"", b"group"),
            Err(NullifierDerivationError::EmptyMemberSecret)
        );
        assert_eq!(
            compute_nullifier_v2(b"secret", b""),
            Err(NullifierDerivationError::EmptyGroupId)
        );
    }

    #[test]
    fn explicit_profile_never_silently_equates_v1_and_v2() {
        let v1 = compute_nullifier_with_profile(
            NullifierDerivationProfile::LegacyDelimiterV1,
            b"alice_secret",
            b"care_circle_42",
        )
        .expect("v1");
        let v2 = compute_nullifier_with_profile(
            NullifierDerivationProfile::LengthPrefixedV2,
            b"alice_secret",
            b"care_circle_42",
        )
        .expect("v2");
        assert_ne!(v1, v2);
    }

    #[test]
    fn different_secrets_and_groups_change_v2_nullifier() {
        let alice = compute_nullifier_v2(b"alice_secret", b"circle_1").unwrap();
        let bob = compute_nullifier_v2(b"bob_secret", b"circle_1").unwrap();
        let other_group = compute_nullifier_v2(b"alice_secret", b"circle_2").unwrap();
        assert_ne!(alice, bob);
        assert_ne!(alice, other_group);
    }

    #[test]
    fn nullifier_with_merkle_membership_is_structural_only() {
        let members: Vec<&[u8]> = vec![
            b"alice_secret",
            b"bob_secret",
            b"carol_secret",
            b"dave_secret",
        ];
        let leaves: Vec<[u8; 32]> = members.iter().map(|s| compute_member_leaf(*s)).collect();
        let root = compute_merkle_root(&leaves);

        let alice_leaf = compute_member_leaf(b"alice_secret");
        let alice_path = generate_merkle_path(&leaves, 0);
        assert!(verify_merkle_path(&alice_leaf, &alice_path, &root));

        let nullifier = compute_nullifier_v2(b"alice_secret", b"care_circle_42").unwrap();
        let proof = NullifierProof {
            nullifier,
            group_root: root,
            proof_bytes: vec![1, 2, 3],
            domain_tag: "ZTML:Hearth:CircleMembership:v1".to_string(),
        };
        assert!(validate_nullifier_proof_structure(&proof).is_ok());
    }

    #[test]
    fn in_memory_double_use_helper_marks_once() {
        let mut nullifier_set = NullifierSet::new();
        let n = compute_nullifier_v2(b"alice", b"vote_proposal_1").unwrap();
        assert!(nullifier_set.check_and_mark(&n));
        assert!(!nullifier_set.check_and_mark(&n));
        assert!(nullifier_set.is_used(&n));
    }

    #[test]
    fn non_member_cannot_reuse_another_leaf_path() {
        let members: Vec<&[u8]> = vec![b"alice", b"bob", b"carol"];
        let leaves: Vec<[u8; 32]> = members.iter().map(|s| compute_member_leaf(*s)).collect();
        let root = compute_merkle_root(&leaves);
        let eve_leaf = compute_member_leaf(b"eve_secret");
        let alice_path = generate_merkle_path(&leaves, 0);
        assert!(!verify_merkle_path(&eve_leaf, &alice_path, &root));
    }

    #[test]
    fn proof_structure_validation_rejects_obvious_malformed_envelopes() {
        let valid = NullifierProof {
            nullifier: [0xAA; 32],
            group_root: [0xBB; 32],
            proof_bytes: vec![1],
            domain_tag: "ZTML:Test:v1".to_string(),
        };
        assert!(validate_nullifier_proof_structure(&valid).is_ok());

        let zero_null = NullifierProof {
            nullifier: [0; 32],
            ..valid.clone()
        };
        assert!(validate_nullifier_proof_structure(&zero_null).is_err());
    }
}
