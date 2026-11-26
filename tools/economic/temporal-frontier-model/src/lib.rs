#![forbid(unsafe_code)]

//! Structural model for verifier-relative, owner-scoped causal frontiers.
//!
//! This crate deliberately separates:
//! - owner-local sequence/coverage;
//! - verifier identity and policy epoch;
//! - partial-order comparison of frontier cuts;
//! - declared receipt inclusion evidence.
//!
//! It does NOT verify cryptographic Merkle proofs, signatures, checkpoint
//! ancestry, Holochain DHT state, or source authenticity. "DeclaredIncluded"
//! means the witness fields are structurally consistent, not cryptographically
//! verified. Production callers must verify proof bytes independently first.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_rfc9162_proof_verifier::{
    Hash32, consistency_proof, empty_tree_hash, inclusion_proof, sha256,
    tree_hash, verify_consistency_proof, verify_entry_inclusion_proof,
};

const MAX_ID_BYTES: usize = 1_024;
const MAX_PROOF_NODES: usize = 65;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SequenceRangeV1 {
    pub first: u64,
    pub last: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CheckpointInclusionDeclarationV1 {
    pub checkpoint_root_digest: String,
    pub owner_root_digest: String,
    pub leaf_index: u64,
    pub tree_size: u64,
    pub inclusion_path: Vec<Hash32>,
    pub proof_ref: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerCutV1 {
    pub owner_ref: String,
    /// Stable identity for one append-only owner history.
    pub history_ref: String,
    /// Highest owner-local sequence allocated/represented by this cut.
    pub sequence: u64,
    /// Number of receipt leaves committed under root_digest. This may be
    /// smaller than sequence when explicit owner-local gaps exist.
    pub tree_size: u64,
    /// Root of this owner's receipt log at this sequence cut.
    pub root_digest: String,
    /// Structural declaration binding the owner root to the frontier root.
    /// Proof bytes are not verified by this crate.
    pub checkpoint_inclusion: CheckpointInclusionDeclarationV1,
    /// Known missing or unresolved sequence ranges at or below `sequence`.
    pub gaps: Vec<SequenceRangeV1>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerifierFrontierV1 {
    pub frontier_id: String,
    pub verifier_ref: String,
    pub policy_epoch_ref: String,
    /// Aggregate checkpoint/root reference. Its cryptographic meaning is
    /// intentionally not interpreted by this structural model.
    pub checkpoint_root_digest: String,
    pub parent_frontier_ids: Vec<String>,
    pub owner_cuts: Vec<OwnerCutV1>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerPositionV1 {
    pub owner_ref: String,
    pub sequence: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReceiptInclusionDeclarationV1 {
    pub owner_root_digest: String,
    pub leaf_index: u64,
    pub tree_size: u64,
    pub inclusion_path: Vec<Hash32>,
    pub proof_ref: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KnowledgeReceiptV1 {
    pub receipt_id: String,
    pub verifier_ref: String,
    pub assertion_id: String,
    pub assertion_digest: String,
    /// Owner of the verifier's receipt-acceptance log, not necessarily the
    /// source owner that authored the underlying assertion.
    pub owner_ref: String,
    /// Position of this acceptance receipt in that owner's append-only log.
    pub owner_sequence: u64,
    pub causal_dependencies: Vec<OwnerPositionV1>,
    pub receipt_digest: String,
    pub inclusion: Option<ReceiptInclusionDeclarationV1>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerConsistencyLinkV1 {
    pub owner_ref: String,
    pub history_ref: String,
    pub ancestor_root_digest: String,
    pub descendant_root_digest: String,
    pub first_tree_size: u64,
    pub second_tree_size: u64,
    pub consistency_path: Vec<Hash32>,
    pub consistency_proof_ref: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FrontierRelationV1 {
    Equal,
    Before,
    After,
    Concurrent,
    Unknown,
    Conflict,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReceiptVisibilityV1 {
    /// RFC 9162 SHA-256 inclusion path recomputes to the supplied
    /// owner-history root. The root's signature, log identity and trust are
    /// not authenticated by this crate.
    MerkleInclusionProofValid,
    /// The receipt sequence is beyond the highest sequence in this cut and
    /// there are no declared gaps in the represented prefix. This is not a
    /// global absence or "verifier never knew" proof.
    BeyondCoveredCut,
    /// Missing owner coverage, a gap, missing proof material, or an uncovered
    /// causal dependency prevents a safe conclusion.
    Indeterminate,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FrontierModelErrorV1 {
    EmptyIdentifier,
    IdentifierTooLong,
    InvalidDigest,
    DuplicateOwnerCut,
    DuplicateParent,
    SelfParent,
    InvalidGap,
    UnsortedOrOverlappingGaps,
    ForeignVerifier,
    DuplicateDependency,
    InvalidCausalDependency,
    InvalidReceiptSequence,
    InvalidInclusionDeclaration,
    TextTooLong,
    InvalidCheckpointProof,
    InvalidReceiptProof,
    InvalidConsistencyProof,
    ReceiptDigestMismatch,
    EmptyOwnerDomain,
}

impl OwnerCutV1 {
    pub fn validate(&self) -> Result<(), FrontierModelErrorV1> {
        validate_id(&self.owner_ref)?;
        validate_id(&self.history_ref)?;
        validate_digest(&self.root_digest)?;
        validate_digest(&self.checkpoint_inclusion.checkpoint_root_digest)?;
        validate_digest(&self.checkpoint_inclusion.owner_root_digest)?;
        validate_id(&self.checkpoint_inclusion.proof_ref)?;
        if self.checkpoint_inclusion.owner_root_digest != self.root_digest
            || self.checkpoint_inclusion.tree_size == 0
            || self.checkpoint_inclusion.leaf_index
                >= self.checkpoint_inclusion.tree_size
            || self.checkpoint_inclusion.inclusion_path.len() > MAX_PROOF_NODES
        {
            return Err(FrontierModelErrorV1::InvalidInclusionDeclaration);
        }

        let mut previous_last = 0_u64;
        let mut missing_count = 0_u64;
        for (index, gap) in self.gaps.iter().enumerate() {
            if gap.first == 0 || gap.first > gap.last || gap.last > self.sequence {
                return Err(FrontierModelErrorV1::InvalidGap);
            }
            if index > 0 && gap.first <= previous_last.saturating_add(1) {
                return Err(FrontierModelErrorV1::UnsortedOrOverlappingGaps);
            }
            previous_last = gap.last;
            let gap_len = gap
                .last
                .checked_sub(gap.first)
                .and_then(|width| width.checked_add(1))
                .ok_or(FrontierModelErrorV1::InvalidGap)?;
            missing_count = missing_count
                .checked_add(gap_len)
                .ok_or(FrontierModelErrorV1::InvalidGap)?;
        }
        if self.tree_size.checked_add(missing_count) != Some(self.sequence) {
            return Err(FrontierModelErrorV1::InvalidGap);
        }
        if self.tree_size == 0 && parse_digest(&self.root_digest)? != empty_tree_hash() {
            return Err(FrontierModelErrorV1::InvalidDigest);
        }
        Ok(())
    }

    /// Map an owner-local sequence to its zero-based committed leaf index.
    /// Returns None for a declared gap or a sequence outside this cut.
    pub fn leaf_index_for_sequence(&self, sequence: u64) -> Option<u64> {
        self.validate().ok()?;
        if sequence == 0
            || sequence > self.sequence
            || self.gaps.iter().any(|gap| gap.first <= sequence && sequence <= gap.last)
        {
            return None;
        }
        let missing_before = self.gaps.iter().try_fold(0_u64, |total, gap| {
            if gap.last < sequence {
                let length = gap.last.checked_sub(gap.first)?.checked_add(1)?;
                total.checked_add(length)
            } else {
                Some(total)
            }
        })?;
        sequence.checked_sub(1)?.checked_sub(missing_before)
    }

    /// True only when the cut reaches `through` and declares no gaps in
    /// that prefix. This is structural gap accounting, not proof that the
    /// root, inventory, or coverage declaration is authentic.
    pub fn prefix_has_no_declared_gaps_through(&self, through: u64) -> bool {
        self.validate().is_ok() && self.prefix_has_no_declared_gaps_after_validation(through)
    }

    fn prefix_has_no_declared_gaps_after_validation(&self, through: u64) -> bool {
        through <= self.sequence && !self.gaps.iter().any(|gap| gap.first <= through)
    }

    pub fn contains_unresolved_gap(&self, sequence: u64) -> bool {
        self.gaps
            .iter()
            .any(|gap| gap.first <= sequence && sequence <= gap.last)
    }
}

impl VerifierFrontierV1 {
    pub fn validate(&self) -> Result<(), FrontierModelErrorV1> {
        validate_id(&self.frontier_id)?;
        validate_id(&self.verifier_ref)?;
        validate_id(&self.policy_epoch_ref)?;
        validate_digest(&self.checkpoint_root_digest)?;

        let mut parents = BTreeSet::new();
        for parent in &self.parent_frontier_ids {
            validate_id(parent)?;
            if parent == &self.frontier_id {
                return Err(FrontierModelErrorV1::SelfParent);
            }
            if !parents.insert(parent) {
                return Err(FrontierModelErrorV1::DuplicateParent);
            }
        }

        if self.owner_cuts.is_empty() {
            return Err(FrontierModelErrorV1::EmptyOwnerDomain);
        }

        let mut owners = BTreeSet::new();
        let mut checkpoint_indices = BTreeSet::new();
        for cut in &self.owner_cuts {
            cut.validate()?;
            if cut.checkpoint_inclusion.checkpoint_root_digest
                != self.checkpoint_root_digest
                || cut.checkpoint_inclusion.tree_size != self.owner_cuts.len() as u64
            {
                return Err(FrontierModelErrorV1::InvalidInclusionDeclaration);
            }
            if !owners.insert(cut.owner_ref.as_str()) {
                return Err(FrontierModelErrorV1::DuplicateOwnerCut);
            }
            if !checkpoint_indices.insert(cut.checkpoint_inclusion.leaf_index) {
                return Err(FrontierModelErrorV1::DuplicateOwnerCut);
            }
            let entry = canonical_owner_cut_entry_v1(
                cut, &self.verifier_ref, &self.policy_epoch_ref,
            )?;
            let checkpoint_root = parse_digest(&self.checkpoint_root_digest)?;
            verify_entry_inclusion_proof(
                &entry,
                cut.checkpoint_inclusion.leaf_index,
                cut.checkpoint_inclusion.tree_size,
                &cut.checkpoint_inclusion.inclusion_path,
                &checkpoint_root,
            )
            .map_err(|_| FrontierModelErrorV1::InvalidCheckpointProof)?;
        }
        if checkpoint_indices.len() != self.owner_cuts.len()
            || checkpoint_indices
                .iter()
                .copied()
                .enumerate()
                .any(|(expected, actual)| expected as u64 != actual)
        {
            return Err(FrontierModelErrorV1::InvalidCheckpointProof);
        }
        Ok(())
    }

    pub fn owner_cut(&self, owner_ref: &str) -> Option<&OwnerCutV1> {
        self.owner_cuts
            .iter()
            .find(|cut| cut.owner_ref == owner_ref)
    }

    /// Does this frontier's owner cut reach this sequence without declared
    /// gaps? This does not prove global completeness or absence: callers must
    /// reconcile the accepted-receipt inventory and authenticate the tree head.
    pub fn prefix_has_no_declared_gaps(&self, owner_ref: &str, through: u64) -> bool {
        self.validate().is_ok()
            && self
                .owner_cut(owner_ref)
                .is_some_and(|cut| cut.prefix_has_no_declared_gaps_after_validation(through))
    }

    pub fn classify_receipt(
        &self,
        receipt: &KnowledgeReceiptV1,
    ) -> Result<ReceiptVisibilityV1, FrontierModelErrorV1> {
        self.validate()?;
        if receipt.verifier_ref != self.verifier_ref {
            return Err(FrontierModelErrorV1::ForeignVerifier);
        }
        receipt.validate()?;

        let Some(owner_cut) = self.owner_cut(&receipt.owner_ref) else {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        };

        if receipt.owner_sequence > owner_cut.sequence {
            return Ok(if owner_cut.prefix_has_no_declared_gaps_after_validation(owner_cut.sequence) {
                ReceiptVisibilityV1::BeyondCoveredCut
            } else {
                ReceiptVisibilityV1::Indeterminate
            });
        }

        if !owner_cut.prefix_has_no_declared_gaps_after_validation(receipt.owner_sequence) {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        }

        for dependency in &receipt.causal_dependencies {
            let Some(dependency_cut) = self.owner_cut(&dependency.owner_ref) else {
                return Ok(ReceiptVisibilityV1::Indeterminate);
            };
            if !dependency_cut.prefix_has_no_declared_gaps_after_validation(dependency.sequence) {
                return Ok(ReceiptVisibilityV1::Indeterminate);
            }
        }

        let Some(witness) = &receipt.inclusion else {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        };
        let Some(expected_leaf_index) =
            owner_cut.leaf_index_for_sequence(receipt.owner_sequence)
        else {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        };
        if witness.owner_root_digest != owner_cut.root_digest
            || witness.tree_size != owner_cut.tree_size
            || witness.leaf_index != expected_leaf_index
        {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        }

        let owner_root = parse_digest(&owner_cut.root_digest)?;
        let entry = canonical_knowledge_receipt_entry_v1(receipt)?;
        verify_entry_inclusion_proof(
            &entry,
            witness.leaf_index,
            witness.tree_size,
            &witness.inclusion_path,
            &owner_root,
        )
        .map_err(|_| FrontierModelErrorV1::InvalidReceiptProof)?;
        Ok(ReceiptVisibilityV1::MerkleInclusionProofValid)
    }

    /// Compare two verifier frontiers over the same policy epoch and owner
    /// domain. A changed owner root needs an RFC 9162 append-only consistency
    /// proof. Proofs establish relations between supplied roots; callers must
    /// still authenticate the tree heads and verifier identity.
    pub fn compare(
        &self,
        other: &Self,
        consistency_links: &[OwnerConsistencyLinkV1],
    ) -> Result<FrontierRelationV1, FrontierModelErrorV1> {
        self.validate()?;
        other.validate()?;
        if self.verifier_ref != other.verifier_ref {
            return Err(FrontierModelErrorV1::ForeignVerifier);
        }
        if self.policy_epoch_ref != other.policy_epoch_ref {
            return Ok(FrontierRelationV1::Unknown);
        }

        let left: BTreeMap<&str, &OwnerCutV1> = self
            .owner_cuts
            .iter()
            .map(|cut| (cut.owner_ref.as_str(), cut))
            .collect();
        let right: BTreeMap<&str, &OwnerCutV1> = other
            .owner_cuts
            .iter()
            .map(|cut| (cut.owner_ref.as_str(), cut))
            .collect();

        if left.keys().ne(right.keys()) {
            return Ok(FrontierRelationV1::Unknown);
        }

        let mut consistency_index = BTreeMap::new();
        for link in consistency_links {
            validate_id(&link.owner_ref)?;
            validate_id(&link.history_ref)?;
            validate_digest(&link.ancestor_root_digest)?;
            validate_digest(&link.descendant_root_digest)?;
            validate_id(&link.consistency_proof_ref)?;
            if link.consistency_path.len() > MAX_PROOF_NODES {
                return Err(FrontierModelErrorV1::InvalidConsistencyProof);
            }
            consistency_index.insert(
                (
                    link.owner_ref.as_str(),
                    link.history_ref.as_str(),
                    link.ancestor_root_digest.as_str(),
                    link.descendant_root_digest.as_str(),
                ),
                link,
            );
        }

        let mut before = false;
        let mut after = false;
        for (owner_ref, left_cut) in &left {
            let left_cut = *left_cut;
            let Some(right_cut) = right.get(*owner_ref).copied() else {
                return Ok(FrontierRelationV1::Unknown);
            };
            if !left_cut.gaps.is_empty() || !right_cut.gaps.is_empty() {
                return Ok(FrontierRelationV1::Unknown);
            }
            if left_cut.history_ref != right_cut.history_ref {
                return Ok(FrontierRelationV1::Conflict);
            }
            if left_cut.sequence == right_cut.sequence {
                if left_cut.root_digest != right_cut.root_digest {
                    return Ok(FrontierRelationV1::Conflict);
                }
                continue;
            }

            let (ancestor, descendant, direction) = if left_cut.sequence < right_cut.sequence {
                (left_cut, right_cut, FrontierRelationV1::Before)
            } else {
                (right_cut, left_cut, FrontierRelationV1::After)
            };
            let Some(link) = consistency_index.get(&(
                *owner_ref,
                ancestor.history_ref.as_str(),
                ancestor.root_digest.as_str(),
                descendant.root_digest.as_str(),
            )) else {
                return Ok(FrontierRelationV1::Unknown);
            };
            if link.first_tree_size != ancestor.tree_size
                || link.second_tree_size != descendant.tree_size
            {
                return Ok(FrontierRelationV1::Unknown);
            }
            let ancestor_root = parse_digest(&ancestor.root_digest)?;
            let descendant_root = parse_digest(&descendant.root_digest)?;
            if verify_consistency_proof(
                link.first_tree_size,
                &ancestor_root,
                link.second_tree_size,
                &descendant_root,
                &link.consistency_path,
            )
            .is_err()
            {
                return Ok(FrontierRelationV1::Unknown);
            }
            match direction {
                FrontierRelationV1::Before => before = true,
                FrontierRelationV1::After => after = true,
                _ => {}
            }
        }

        if !before
            && !after
            && self.checkpoint_root_digest != other.checkpoint_root_digest
        {
            return Ok(FrontierRelationV1::Unknown);
        }

        Ok(match (before, after) {
            (false, false) => FrontierRelationV1::Equal,
            (true, false) => FrontierRelationV1::Before,
            (false, true) => FrontierRelationV1::After,
            (true, true) => FrontierRelationV1::Concurrent,
        })
    }
}

impl KnowledgeReceiptV1 {
    pub fn validate(&self) -> Result<(), FrontierModelErrorV1> {
        validate_id(&self.receipt_id)?;
        validate_id(&self.verifier_ref)?;
        validate_id(&self.assertion_id)?;
        validate_digest(&self.assertion_digest)?;
        validate_id(&self.owner_ref)?;
        validate_digest(&self.receipt_digest)?;
        if self.owner_sequence == 0 {
            return Err(FrontierModelErrorV1::InvalidReceiptSequence);
        }

        let mut dependencies = BTreeSet::new();
        for dependency in &self.causal_dependencies {
            validate_id(&dependency.owner_ref)?;
            if dependency.sequence == 0 {
                return Err(FrontierModelErrorV1::InvalidCausalDependency);
            }
            if !dependencies.insert((dependency.owner_ref.as_str(), dependency.sequence)) {
                return Err(FrontierModelErrorV1::DuplicateDependency);
            }
            if dependency.owner_ref == self.owner_ref
                && dependency.sequence >= self.owner_sequence
            {
                return Err(FrontierModelErrorV1::InvalidCausalDependency);
            }
        }

        if let Some(witness) = &self.inclusion {
            validate_digest(&witness.owner_root_digest)?;
            validate_id(&witness.proof_ref)?;
            if witness.tree_size == 0
                || witness.leaf_index >= witness.tree_size
                || witness.inclusion_path.len() > MAX_PROOF_NODES
            {
                return Err(FrontierModelErrorV1::InvalidInclusionDeclaration);
            }
        }
        let expected_digest = format!(
            "sha256:{}",
            hex_lower(&sha256(&canonical_knowledge_receipt_entry_v1(self)?))
        );
        if expected_digest != self.receipt_digest {
            return Err(FrontierModelErrorV1::ReceiptDigestMismatch);
        }
        Ok(())
    }
}

/// Canonical owner-cut entry bytes committed by the aggregate checkpoint.
/// Inclusion-proof metadata is excluded to avoid self-reference.
pub fn canonical_owner_cut_entry_v1(
    cut: &OwnerCutV1,
    verifier_ref: &str,
    policy_epoch_ref: &str,
) -> Result<Vec<u8>, FrontierModelErrorV1> {
    validate_id(&cut.owner_ref)?;
    validate_id(&cut.history_ref)?;
    validate_id(verifier_ref)?;
    validate_id(policy_epoch_ref)?;
    validate_digest(&cut.root_digest)?;
    let gap_count = u32::try_from(cut.gaps.len())
        .map_err(|_| FrontierModelErrorV1::TextTooLong)?;
    let mut bytes = b"MYCELIX-OWNER-CUT-V1\0".to_vec();
    append_string(&mut bytes, verifier_ref)?;
    append_string(&mut bytes, policy_epoch_ref)?;
    append_string(&mut bytes, &cut.owner_ref)?;
    append_string(&mut bytes, &cut.history_ref)?;
    bytes.extend_from_slice(&cut.sequence.to_be_bytes());
    bytes.extend_from_slice(&cut.tree_size.to_be_bytes());
    append_string(&mut bytes, &cut.root_digest)?;
    bytes.extend_from_slice(&gap_count.to_be_bytes());
    for gap in &cut.gaps {
        bytes.extend_from_slice(&gap.first.to_be_bytes());
        bytes.extend_from_slice(&gap.last.to_be_bytes());
    }
    Ok(bytes)
}

/// Canonical verifier-acceptance receipt bytes. Inclusion proof fields and the
/// receipt digest itself are excluded; causal dependencies are sorted so a
/// permutation of an otherwise identical dependency set has one encoding.
pub fn canonical_knowledge_receipt_entry_v1(
    receipt: &KnowledgeReceiptV1,
) -> Result<Vec<u8>, FrontierModelErrorV1> {
    validate_id(&receipt.receipt_id)?;
    validate_id(&receipt.verifier_ref)?;
    validate_id(&receipt.assertion_id)?;
    validate_digest(&receipt.assertion_digest)?;
    validate_id(&receipt.owner_ref)?;
    if receipt.owner_sequence == 0 {
        return Err(FrontierModelErrorV1::InvalidReceiptSequence);
    }

    let mut dependencies: Vec<(&str, u64)> = receipt
        .causal_dependencies
        .iter()
        .map(|dependency| {
            validate_id(&dependency.owner_ref)?;
            if dependency.sequence == 0 {
                return Err(FrontierModelErrorV1::InvalidCausalDependency);
            }
            Ok((dependency.owner_ref.as_str(), dependency.sequence))
        })
        .collect::<Result<_, FrontierModelErrorV1>>()?;
    dependencies.sort_unstable();

    for pair in dependencies.windows(2) {
        if pair[0] == pair[1] {
            return Err(FrontierModelErrorV1::DuplicateDependency);
        }
    }

    let dependency_count = u32::try_from(dependencies.len())
        .map_err(|_| FrontierModelErrorV1::TextTooLong)?;
    let mut bytes = b"MYCELIX-KNOWLEDGE-RECEIPT-V1\0".to_vec();
    append_string(&mut bytes, &receipt.receipt_id)?;
    append_string(&mut bytes, &receipt.verifier_ref)?;
    append_string(&mut bytes, &receipt.assertion_id)?;
    append_string(&mut bytes, &receipt.assertion_digest)?;
    append_string(&mut bytes, &receipt.owner_ref)?;
    bytes.extend_from_slice(&receipt.owner_sequence.to_be_bytes());
    bytes.extend_from_slice(&dependency_count.to_be_bytes());
    for (owner_ref, sequence) in dependencies {
        append_string(&mut bytes, owner_ref)?;
        bytes.extend_from_slice(&sequence.to_be_bytes());
    }
    Ok(bytes)
}

fn append_string(
    output: &mut Vec<u8>,
    value: &str,
) -> Result<(), FrontierModelErrorV1> {
    let length = u32::try_from(value.len())
        .map_err(|_| FrontierModelErrorV1::TextTooLong)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
    Ok(())
}

fn parse_digest(value: &str) -> Result<Hash32, FrontierModelErrorV1> {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return Err(FrontierModelErrorV1::InvalidDigest);
    };
    if hex.len() != 64 {
        return Err(FrontierModelErrorV1::InvalidDigest);
    }
    let mut output = [0_u8; 32];
    for (index, pair) in hex.as_bytes().chunks_exact(2).enumerate() {
        let high = hex_nibble(pair[0]).ok_or(FrontierModelErrorV1::InvalidDigest)?;
        let low = hex_nibble(pair[1]).ok_or(FrontierModelErrorV1::InvalidDigest)?;
        output[index] = (high << 4) | low;
    }
    Ok(output)
}

fn hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        _ => None,
    }
}

fn hex_lower(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

fn validate_id(value: &str) -> Result<(), FrontierModelErrorV1> {
    if value.trim().is_empty() {
        return Err(FrontierModelErrorV1::EmptyIdentifier);
    }
    if value.len() > MAX_ID_BYTES {
        return Err(FrontierModelErrorV1::IdentifierTooLong);
    }
    Ok(())
}

fn validate_digest(value: &str) -> Result<(), FrontierModelErrorV1> {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return Err(FrontierModelErrorV1::InvalidDigest);
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(FrontierModelErrorV1::InvalidDigest);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const VERIFIER: &str = "did:mycelix:verifier:finance-v1";

    fn digest(byte: char) -> String {
        format!("sha256:{}", byte.to_string().repeat(64))
    }

    fn owner_log_entries(
        owner: &str,
        sequence: u64,
        gaps: &[SequenceRangeV1],
    ) -> Vec<Vec<u8>> {
        (1..=sequence)
            .filter(|position| {
                !gaps.iter().any(|gap| gap.first <= *position && *position <= gap.last)
            })
            .map(|position| format!("MYCELIX-TEST-OWNER-LOG:{owner}:{position}").into_bytes())
            .collect()
    }

    fn cut(owner: &str, sequence: u64, _label: char) -> OwnerCutV1 {
        let entries = owner_log_entries(owner, sequence, &[]);
        let root_digest = format!(
            "sha256:{}",
            hex_lower(&tree_hash(&entries).unwrap())
        );
        OwnerCutV1 {
            owner_ref: owner.into(),
            history_ref: format!("history:{owner}"),
            sequence,
            tree_size: entries.len() as u64,
            root_digest: root_digest.clone(),
            checkpoint_inclusion: CheckpointInclusionDeclarationV1 {
                checkpoint_root_digest: digest('f'),
                owner_root_digest: root_digest,
                leaf_index: 0,
                tree_size: 1,
                inclusion_path: Vec::new(),
                proof_ref: "proof:owner-checkpoint-binding".into(),
            },
            gaps: Vec::new(),
        }
    }

    fn frontier(id: &str, mut owner_cuts: Vec<OwnerCutV1>) -> VerifierFrontierV1 {
        owner_cuts.sort_by(|left, right| left.owner_ref.cmp(&right.owner_ref));
        let entries: Vec<Vec<u8>> = owner_cuts
            .iter()
            .map(|cut| {
                canonical_owner_cut_entry_v1(cut, VERIFIER, "policy-epoch:1").unwrap()
            })
            .collect();
        let checkpoint_root_digest =
            format!("sha256:{}", hex_lower(&tree_hash(&entries).unwrap()));
        for (index, cut) in owner_cuts.iter_mut().enumerate() {
            cut.checkpoint_inclusion = CheckpointInclusionDeclarationV1 {
                checkpoint_root_digest: checkpoint_root_digest.clone(),
                owner_root_digest: cut.root_digest.clone(),
                leaf_index: index as u64,
                tree_size: entries.len() as u64,
                inclusion_path: inclusion_proof(&entries, index as u64).unwrap(),
                proof_ref: format!("proof:owner-checkpoint:{index}"),
            };
        }
        VerifierFrontierV1 {
            frontier_id: id.into(),
            verifier_ref: VERIFIER.into(),
            policy_epoch_ref: "policy-epoch:1".into(),
            checkpoint_root_digest,
            parent_frontier_ids: Vec::new(),
            owner_cuts,
        }
    }

    fn receipt(sequence: u64, owner_ref: &str) -> KnowledgeReceiptV1 {
        let mut receipt = KnowledgeReceiptV1 {
            receipt_id: format!("receipt:{sequence}"),
            verifier_ref: VERIFIER.into(),
            assertion_id: "assertion:revocation".into(),
            assertion_digest: digest('b'),
            owner_ref: owner_ref.into(),
            owner_sequence: sequence,
            causal_dependencies: Vec::new(),
            receipt_digest: digest('0'),
            inclusion: None,
        };
        receipt.receipt_digest = format!(
            "sha256:{}",
            hex_lower(&sha256(&canonical_knowledge_receipt_entry_v1(&receipt).unwrap()))
        );
        receipt
    }

    fn refresh_receipt_digest(receipt: &mut KnowledgeReceiptV1) {
        receipt.receipt_digest = format!(
            "sha256:{}",
            hex_lower(&sha256(&canonical_knowledge_receipt_entry_v1(receipt).unwrap()))
        );
    }

    fn attach_receipt_proof(
        receipt: &mut KnowledgeReceiptV1,
        owner_cut: &mut OwnerCutV1,
    ) {
        assert!(receipt.owner_sequence > 0);
        assert!(receipt.owner_sequence <= owner_cut.sequence);
        refresh_receipt_digest(receipt);
        let mut entries = owner_log_entries(
            &owner_cut.owner_ref, owner_cut.sequence, &owner_cut.gaps,
        );
        let leaf_index = owner_cut
            .leaf_index_for_sequence(receipt.owner_sequence)
            .expect("receipt sequence is not a declared gap");
        entries[leaf_index as usize] =
            canonical_knowledge_receipt_entry_v1(receipt).unwrap();
        let root = tree_hash(&entries).unwrap();
        owner_cut.tree_size = entries.len() as u64;
        owner_cut.root_digest = format!("sha256:{}", hex_lower(&root));
        receipt.inclusion = Some(ReceiptInclusionDeclarationV1 {
            owner_root_digest: owner_cut.root_digest.clone(),
            leaf_index,
            tree_size: owner_cut.tree_size,
            inclusion_path: inclusion_proof(&entries, receipt.owner_sequence - 1).unwrap(),
            proof_ref: format!("proof:receipt:{}", receipt.receipt_id),
        });
    }

    fn consistency_link(
        owner_ref: &str,
        first_tree_size: u64,
        second_tree_size: u64,
    ) -> OwnerConsistencyLinkV1 {
        let entries = owner_log_entries(owner_ref, second_tree_size, &[]);
        let ancestor_root = tree_hash(&entries[..first_tree_size as usize]).unwrap();
        let descendant_root = tree_hash(&entries).unwrap();
        OwnerConsistencyLinkV1 {
            owner_ref: owner_ref.into(),
            history_ref: format!("history:{owner_ref}"),
            ancestor_root_digest: format!("sha256:{}", hex_lower(&ancestor_root)),
            descendant_root_digest: format!("sha256:{}", hex_lower(&descendant_root)),
            first_tree_size,
            second_tree_size,
            consistency_path: consistency_proof(&entries, first_tree_size).unwrap(),
            consistency_proof_ref: format!("proof:consistency:{owner_ref}:{first_tree_size}:{second_tree_size}"),
        }
    }

    fn vector_oracle(left: [u64; 2], right: [u64; 2]) -> FrontierRelationV1 {
        let mut before = false;
        let mut after = false;
        for (left_position, right_position) in left.into_iter().zip(right) {
            before |= left_position < right_position;
            after |= left_position > right_position;
        }
        match (before, after) {
            (false, false) => FrontierRelationV1::Equal,
            (true, false) => FrontierRelationV1::Before,
            (false, true) => FrontierRelationV1::After,
            (true, true) => FrontierRelationV1::Concurrent,
        }
    }

    fn bounded_frontier(id: &str, positions: [u64; 2]) -> VerifierFrontierV1 {
        frontier(
            id,
            vec![
                cut("owner:a", positions[0], 'a'),
                cut("owner:b", positions[1], 'b'),
            ],
        )
    }

    fn bounded_links(left: [u64; 2], right: [u64; 2]) -> Vec<OwnerConsistencyLinkV1> {
        let mut links = Vec::new();
        for (owner_index, (left_position, right_position)) in
            left.into_iter().zip(right).enumerate()
        {
            let (ancestor, descendant) = if left_position < right_position {
                (left_position, right_position)
            } else if right_position < left_position {
                (right_position, left_position)
            } else {
                continue;
            };
            let owner_ref = if owner_index == 0 { "owner:a" } else { "owner:b" };
            links.push(consistency_link(owner_ref, ancestor, descendant));
        }
        links
    }

    #[test]
    fn exhaustive_two_owner_frontiers_match_independent_vector_oracle() {
        let mut states = Vec::new();
        for owner_a in 0..=2 {
            for owner_b in 0..=2 {
                let positions = [owner_a, owner_b];
                let id = format!("frontier:{}:{}", owner_a, owner_b);
                states.push((positions, bounded_frontier(&id, positions)));
            }
        }

        for (left_positions, left) in &states {
            for (right_positions, right) in &states {
                let expected = vector_oracle(*left_positions, *right_positions);
                let actual = left
                    .compare(right, &bounded_links(*left_positions, *right_positions))
                    .unwrap();
                assert_eq!(
                    actual, expected,
                    "frontier relation mismatch: {left_positions:?} vs {right_positions:?}"
                );

                let inverse = right
                    .compare(left, &bounded_links(*right_positions, *left_positions))
                    .unwrap();
                let inverse_expected = match expected {
                    FrontierRelationV1::Before => FrontierRelationV1::After,
                    FrontierRelationV1::After => FrontierRelationV1::Before,
                    relation => relation,
                };
                assert_eq!(inverse, inverse_expected);
            }
        }

        for (a_positions, a) in &states {
            for (b_positions, b) in &states {
                if vector_oracle(*a_positions, *b_positions) != FrontierRelationV1::Before {
                    continue;
                }
                for (c_positions, c) in &states {
                    if vector_oracle(*b_positions, *c_positions) != FrontierRelationV1::Before {
                        continue;
                    }
                    assert_eq!(
                        a.compare(c, &bounded_links(*a_positions, *c_positions)),
                        Ok(FrontierRelationV1::Before)
                    );
                }
            }
        }
    }

    #[test]
    fn same_owner_prefix_extension_is_before_only_with_consistency_link() {
        let first_cut = cut("owner:grants", 10, 'a');
        let later_cut = cut("owner:grants", 20, 'b');
        let first = frontier("frontier:1", vec![first_cut.clone()]);
        let later = frontier("frontier:2", vec![later_cut.clone()]);
        let link = consistency_link("owner:grants", 10, 20);
        assert_eq!(first.compare(&later, &[link]), Ok(FrontierRelationV1::Before));
    }

    #[test]
    fn crossed_owner_cuts_are_concurrent_not_totally_ordered() {
        let a = frontier("frontier:a", vec![cut("owner:a", 10, 'a'), cut("owner:b", 20, 'b')]);
        let b = frontier("frontier:b", vec![cut("owner:a", 20, 'c'), cut("owner:b", 10, 'd')]);
        let links = vec![
            consistency_link("owner:a", 10, 20),
            consistency_link("owner:b", 10, 20),
        ];
        assert_eq!(a.compare(&b, &links), Ok(FrontierRelationV1::Concurrent));
    }

    #[test]
    fn altered_aggregate_root_without_a_valid_inclusion_path_is_rejected() {
        let a = frontier("frontier:a", vec![cut("owner:a", 10, 'a')]);
        let mut b = frontier("frontier:b", vec![cut("owner:a", 10, 'a')]);
        b.checkpoint_root_digest = digest('e');
        b.owner_cuts[0].checkpoint_inclusion.checkpoint_root_digest = digest('e');
        assert_eq!(
            b.validate(),
            Err(FrontierModelErrorV1::InvalidCheckpointProof)
        );
        assert!(a.validate().is_ok());
    }

    #[test]
    fn equal_sequence_with_different_root_is_a_fork_conflict() {
        let a = frontier("frontier:a", vec![cut("owner:a", 10, 'a')]);
        let mut fork_cut = cut("owner:a", 10, 'a');
        fork_cut.root_digest = digest('b');
        let b = frontier("frontier:b", vec![fork_cut]);
        assert_eq!(a.compare(&b, &[]), Ok(FrontierRelationV1::Conflict));
    }

    #[test]
    fn different_prefixes_without_consistency_proof_are_unknown() {
        let a = frontier("frontier:a", vec![cut("owner:a", 10, 'a')]);
        let b = frontier("frontier:b", vec![cut("owner:a", 20, 'b')]);
        assert_eq!(a.compare(&b, &[]), Ok(FrontierRelationV1::Unknown));
    }

    #[test]
    fn covered_receipt_requires_a_valid_rfc9162_inclusion_proof() {
        let mut owner_cut = cut("owner:grants", 10, 'a');
        let mut receipt = receipt(5, "owner:grants");
        attach_receipt_proof(&mut receipt, &mut owner_cut);
        let frontier = frontier("frontier:1", vec![owner_cut.clone()]);
        assert_eq!(
            frontier.classify_receipt(&receipt),
            Ok(ReceiptVisibilityV1::MerkleInclusionProofValid)
        );

        let mut no_proof = receipt.clone();
        no_proof.inclusion = None;
        assert_eq!(
            frontier.classify_receipt(&no_proof),
            Ok(ReceiptVisibilityV1::Indeterminate)
        );

        let mut wrong_root = receipt;
        wrong_root.inclusion.as_mut().unwrap().owner_root_digest = digest('c');
        assert_eq!(
            frontier.classify_receipt(&wrong_root),
            Ok(ReceiptVisibilityV1::Indeterminate)
        );
    }

    #[test]
    fn receipt_beyond_a_complete_cut_is_not_known() {
        let owner_cut = cut("owner:grants", 10, 'a');
        let frontier = frontier("frontier:1", vec![owner_cut]);
        assert_eq!(
            frontier.classify_receipt(&receipt(11, "owner:grants")),
            Ok(ReceiptVisibilityV1::BeyondCoveredCut)
        );
    }

    #[test]
    fn gaps_make_inclusion_and_absence_indeterminate() {
        let mut owner_cut = cut("owner:grants", 10, 'a');
        owner_cut.gaps.push(SequenceRangeV1 { first: 3, last: 4 });
        owner_cut.tree_size = 8;
        let mut receipt = receipt(5, "owner:grants");
        attach_receipt_proof(&mut receipt, &mut owner_cut);
        assert_eq!(owner_cut.leaf_index_for_sequence(5), Some(2));
        let frontier = frontier("frontier:1", vec![owner_cut.clone()]);
        assert_eq!(
            frontier.classify_receipt(&receipt),
            Ok(ReceiptVisibilityV1::Indeterminate)
        );
        assert!(!frontier.prefix_has_no_declared_gaps("owner:grants", 10));

        // A receipt that claims a sequence inside a declared gap is never
        // assigned a Merkle leaf index.
        let mut gap_receipt = receipt(3, "owner:grants");
        refresh_receipt_digest(&mut gap_receipt);
        assert_eq!(owner_cut.leaf_index_for_sequence(3), None);
        assert_eq!(
            frontier.classify_receipt(&gap_receipt),
            Ok(ReceiptVisibilityV1::Indeterminate)
        );
    }

    #[test]
    fn a_frontier_does_not_accept_another_verifiers_receipt() {
        let owner_cut = cut("owner:grants", 10, 'a');
        let frontier = frontier("frontier:1", vec![owner_cut]);
        let mut receipt = receipt(5, "owner:grants");
        receipt.verifier_ref = "did:mycelix:verifier:other-v1".into();
        assert_eq!(
            frontier.classify_receipt(&receipt),
            Err(FrontierModelErrorV1::ForeignVerifier)
        );
    }

    #[test]
    fn uncovered_causal_dependency_is_indeterminate() {
        let mut owner_cut = cut("owner:grants", 10, 'a');
        let mut receipt = receipt(5, "owner:grants");
        receipt.causal_dependencies.push(OwnerPositionV1 {
            owner_ref: "owner:payments".into(),
            sequence: 2,
        });
        attach_receipt_proof(&mut receipt, &mut owner_cut);
        let frontier = frontier("frontier:1", vec![owner_cut]);
        assert_eq!(
            frontier.classify_receipt(&receipt),
            Ok(ReceiptVisibilityV1::Indeterminate)
        );
    }

    #[test]
    fn empty_owner_log_uses_the_rfc_empty_tree_hash() {
        let candidate = frontier("frontier:empty-owner", vec![cut("owner:empty", 0, 'a')]);
        assert!(candidate.validate().is_ok());
        let empty_cut = candidate.owner_cut("owner:empty").unwrap();
        assert_eq!(empty_cut.tree_size, 0);
        assert_eq!(parse_digest(&empty_cut.root_digest).unwrap(), empty_tree_hash());
    }

    #[test]
    fn checkpoint_inclusion_path_tampering_is_rejected() {
        let mut candidate = frontier("frontier:1", vec![cut("owner:a", 10, 'a')]);
        candidate.owner_cuts[0].checkpoint_inclusion.inclusion_path[0] = sha256(b"tampered");
        assert_eq!(
            candidate.validate(),
            Err(FrontierModelErrorV1::InvalidCheckpointProof)
        );
    }

    #[test]
    fn receipt_inclusion_path_tampering_is_rejected() {
        let mut owner_cut = cut("owner:grants", 10, 'a');
        let mut receipt = receipt(5, "owner:grants");
        attach_receipt_proof(&mut receipt, &mut owner_cut);
        receipt.inclusion.as_mut().unwrap().inclusion_path[0] = sha256(b"tampered");
        let frontier = frontier("frontier:1", vec![owner_cut]);
        assert_eq!(
            frontier.classify_receipt(&receipt),
            Err(FrontierModelErrorV1::InvalidReceiptProof)
        );
    }

    #[test]
    fn consistency_proof_path_tampering_is_unknown() {
        let first = frontier("frontier:first", vec![cut("owner:a", 10, 'a')]);
        let later = frontier("frontier:later", vec![cut("owner:a", 20, 'a')]);
        let mut link = consistency_link("owner:a", 10, 20);
        link.consistency_path[0] = sha256(b"tampered");
        assert_eq!(
            first.compare(&later, &[link]),
            Ok(FrontierRelationV1::Unknown)
        );
    }

    #[test]
    fn checkpoint_must_bind_each_owner_root() {
        let mut candidate = frontier("frontier:1", vec![cut("owner:a", 10, 'a')]);
        candidate.owner_cuts[0].checkpoint_inclusion.checkpoint_root_digest = digest('e');
        assert_eq!(
            candidate.validate(),
            Err(FrontierModelErrorV1::InvalidInclusionDeclaration)
        );
    }

    #[test]
    fn gaps_make_frontier_comparison_unknown_even_with_sequence_links() {
        let mut left_cut = cut("owner:a", 10, 'a');
        left_cut.gaps.push(SequenceRangeV1 { first: 3, last: 3 });
        let right_cut = cut("owner:a", 20, 'b');
        let left = frontier("frontier:left", vec![left_cut]);
        let right = frontier("frontier:right", vec![right_cut]);
        let link = consistency_link("owner:a", 10, 20);
        assert_eq!(left.compare(&right, &[link]), Ok(FrontierRelationV1::Unknown));
    }

    #[test]
    fn same_owner_dependency_must_precede_its_receipt() {
        let mut candidate = receipt(5, "owner:a");
        candidate.causal_dependencies.push(OwnerPositionV1 {
            owner_ref: "owner:a".into(),
            sequence: 5,
        });
        assert_eq!(
            candidate.validate(),
            Err(FrontierModelErrorV1::InvalidCausalDependency)
        );
    }

    #[test]
    fn receipt_canonical_encoding_is_dependency_order_invariant() {
        let mut first = receipt(5, "owner:grants");
        first.causal_dependencies = vec![
            OwnerPositionV1 { owner_ref: "owner:b".into(), sequence: 4 },
            OwnerPositionV1 { owner_ref: "owner:a".into(), sequence: 2 },
        ];
        let mut second = first.clone();
        second.causal_dependencies.reverse();
        assert_eq!(
            canonical_knowledge_receipt_entry_v1(&first).unwrap(),
            canonical_knowledge_receipt_entry_v1(&second).unwrap()
        );
    }

    #[test]
    fn modified_receipt_payload_fails_digest_validation() {
        let mut candidate = receipt(5, "owner:grants");
        candidate.assertion_id = "assertion:tampered".into();
        assert_eq!(
            candidate.validate(),
            Err(FrontierModelErrorV1::ReceiptDigestMismatch)
        );
    }

    #[test]
    fn invalid_cut_cannot_claim_prefix_coverage() {
        let mut candidate = cut("owner:a", 10, 'a');
        candidate.gaps = vec![
            SequenceRangeV1 { first: 5, last: 4 },
        ];
        assert!(!candidate.prefix_has_no_declared_gaps_through(10));
    }

    #[test]
    fn owner_cuts_and_gap_ranges_must_be_unique_and_ordered() {
        let duplicate = frontier(
            "frontier:1",
            vec![cut("owner:a", 10, 'a'), cut("owner:a", 10, 'a')],
        );
        assert_eq!(
            duplicate.validate(),
            Err(FrontierModelErrorV1::DuplicateOwnerCut)
        );

        let mut invalid = cut("owner:a", 10, 'a');
        invalid.gaps = vec![
            SequenceRangeV1 { first: 4, last: 5 },
            SequenceRangeV1 { first: 5, last: 6 },
        ];
        assert_eq!(
            invalid.validate(),
            Err(FrontierModelErrorV1::UnsortedOrOverlappingGaps)
        );
    }
}
