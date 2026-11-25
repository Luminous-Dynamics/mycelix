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

const MAX_ID_BYTES: usize = 1_024;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SequenceRangeV1 {
    pub first: u64,
    pub last: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CheckpointInclusionDeclarationV1 {
    pub checkpoint_root_digest: String,
    pub owner_root_digest: String,
    pub proof_ref: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OwnerCutV1 {
    pub owner_ref: String,
    /// Stable identity for one append-only owner history.
    pub history_ref: String,
    /// Highest sequence represented by this cut. Sequence zero is an empty
    /// history prefix; non-empty histories conventionally start at one.
    pub sequence: u64,
    /// Root of this owner's history at `sequence`.
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
    pub receipt_leaf_digest: String,
    pub leaf_index: u64,
    pub tree_size: u64,
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
    /// Receipt leaf and owner root references agree structurally. This does
    /// NOT mean that a cryptographic proof was checked.
    DeclaredIncluded,
    /// Frontier coverage is complete through its cut, and the receipt's
    /// sequence is strictly beyond that cut.
    NotKnown,
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
}

impl OwnerCutV1 {
    pub fn validate(&self) -> Result<(), FrontierModelErrorV1> {
        validate_id(&self.owner_ref)?;
        validate_id(&self.history_ref)?;
        validate_digest(&self.root_digest)?;
        validate_digest(&self.checkpoint_inclusion.checkpoint_root_digest)?;
        validate_digest(&self.checkpoint_inclusion.owner_root_digest)?;
        validate_id(&self.checkpoint_inclusion.proof_ref)?;
        if self.checkpoint_inclusion.owner_root_digest != self.root_digest {
            return Err(FrontierModelErrorV1::InvalidInclusionDeclaration);
        }

        let mut previous_last = 0_u64;
        for (index, gap) in self.gaps.iter().enumerate() {
            if gap.first == 0 || gap.first > gap.last || gap.last > self.sequence {
                return Err(FrontierModelErrorV1::InvalidGap);
            }
            if index > 0 && gap.first <= previous_last.saturating_add(1) {
                return Err(FrontierModelErrorV1::UnsortedOrOverlappingGaps);
            }
            previous_last = gap.last;
        }
        Ok(())
    }

    /// True only when every sequence from one through `through` is covered
    /// by this cut. This is structural gap accounting, not proof that the
    /// root or coverage declaration is authentic.
    pub fn prefix_is_complete_through(&self, through: u64) -> bool {
        self.validate().is_ok() && self.prefix_is_complete_after_validation(through)
    }

    fn prefix_is_complete_after_validation(&self, through: u64) -> bool {
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

        let mut owners = BTreeSet::new();
        for cut in &self.owner_cuts {
            cut.validate()?;
            if cut.checkpoint_inclusion.checkpoint_root_digest
                != self.checkpoint_root_digest
            {
                return Err(FrontierModelErrorV1::InvalidInclusionDeclaration);
            }
            if !owners.insert(cut.owner_ref.as_str()) {
                return Err(FrontierModelErrorV1::DuplicateOwnerCut);
            }
        }
        Ok(())
    }

    pub fn owner_cut(&self, owner_ref: &str) -> Option<&OwnerCutV1> {
        self.owner_cuts
            .iter()
            .find(|cut| cut.owner_ref == owner_ref)
    }

    /// Does the frontier structurally cover a complete owner prefix through
    /// this sequence? False means "do not assert absence"; it does not say
    /// whether the missing information is malicious or merely delayed.
    /// True is necessary but not sufficient for an absence claim: callers
    /// must also reconcile the complete accepted-receipt inventory.
    pub fn prefix_coverage_is_complete(&self, owner_ref: &str, through: u64) -> bool {
        self.validate().is_ok()
            && self
                .owner_cut(owner_ref)
                .is_some_and(|cut| cut.prefix_is_complete_after_validation(through))
    }

    pub fn classify_receipt(
        &self,
        receipt: &KnowledgeReceiptV1,
    ) -> Result<ReceiptVisibilityV1, FrontierModelErrorV1> {
        self.validate()?;
        receipt.validate()?;
        if receipt.verifier_ref != self.verifier_ref {
            return Err(FrontierModelErrorV1::ForeignVerifier);
        }

        let Some(owner_cut) = self.owner_cut(&receipt.owner_ref) else {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        };

        if receipt.owner_sequence > owner_cut.sequence {
            return Ok(if owner_cut.prefix_is_complete_after_validation(owner_cut.sequence) {
                ReceiptVisibilityV1::NotKnown
            } else {
                ReceiptVisibilityV1::Indeterminate
            });
        }

        if !owner_cut.prefix_is_complete_after_validation(receipt.owner_sequence) {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        }

        for dependency in &receipt.causal_dependencies {
            let Some(dependency_cut) = self.owner_cut(&dependency.owner_ref) else {
                return Ok(ReceiptVisibilityV1::Indeterminate);
            };
            if !dependency_cut.prefix_is_complete_after_validation(dependency.sequence) {
                return Ok(ReceiptVisibilityV1::Indeterminate);
            }
        }

        let Some(witness) = &receipt.inclusion else {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        };
        if witness.owner_root_digest != owner_cut.root_digest
            || witness.receipt_leaf_digest != receipt.receipt_digest
            || witness.leaf_index >= witness.tree_size
        {
            return Ok(ReceiptVisibilityV1::Indeterminate);
        }
        Ok(ReceiptVisibilityV1::DeclaredIncluded)
    }

    /// Compare two verifier frontiers over the same policy epoch and owner
    /// domain. A changed owner root needs an explicit consistency-link
    /// declaration. Link shape is checked; proof bytes are NOT verified.
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

        let mut consistency_index = BTreeSet::new();
        for link in consistency_links {
            validate_id(&link.owner_ref)?;
            validate_id(&link.history_ref)?;
            validate_digest(&link.ancestor_root_digest)?;
            validate_digest(&link.descendant_root_digest)?;
            validate_id(&link.consistency_proof_ref)?;
            consistency_index.insert((
                link.owner_ref.as_str(),
                link.history_ref.as_str(),
                link.ancestor_root_digest.as_str(),
                link.descendant_root_digest.as_str(),
            ));
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
            let link_exists = consistency_index.contains(&(
                *owner_ref,
                ancestor.history_ref.as_str(),
                ancestor.root_digest.as_str(),
                descendant.root_digest.as_str(),
            ));
            if !link_exists {
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
            validate_digest(&witness.receipt_leaf_digest)?;
            validate_id(&witness.proof_ref)?;
            if witness.tree_size == 0 || witness.leaf_index >= witness.tree_size {
                return Err(FrontierModelErrorV1::InvalidInclusionDeclaration);
            }
        }
        Ok(())
    }
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

    fn cut(owner: &str, sequence: u64, root: char) -> OwnerCutV1 {
        OwnerCutV1 {
            owner_ref: owner.into(),
            history_ref: format!("history:{owner}"),
            sequence,
            root_digest: digest(root),
            checkpoint_inclusion: CheckpointInclusionDeclarationV1 {
                checkpoint_root_digest: digest('f'),
                owner_root_digest: digest(root),
                proof_ref: "proof:owner-checkpoint-binding".into(),
            },
            gaps: Vec::new(),
        }
    }

    fn frontier(id: &str, mut owner_cuts: Vec<OwnerCutV1>) -> VerifierFrontierV1 {
        // Test-only deterministic stand-in: the production adapter must
        // compute and authenticate the actual canonical aggregate root.
        let aggregate_value = id
            .bytes()
            .fold(0_u64, |value, byte| value.wrapping_mul(31).wrapping_add(u64::from(byte)));
        let checkpoint_root_digest = format!("sha256:{aggregate_value:064x}");
        for cut in &mut owner_cuts {
            cut.checkpoint_inclusion.checkpoint_root_digest =
                checkpoint_root_digest.clone();
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

    fn receipt(sequence: u64, owner_cut: &OwnerCutV1) -> KnowledgeReceiptV1 {
        let receipt_digest = digest('a');
        KnowledgeReceiptV1 {
            receipt_id: format!("receipt:{sequence}"),
            verifier_ref: VERIFIER.into(),
            assertion_id: "assertion:revocation".into(),
            assertion_digest: digest('b'),
            owner_ref: owner_cut.owner_ref.clone(),
            owner_sequence: sequence,
            causal_dependencies: Vec::new(),
            receipt_digest: receipt_digest.clone(),
            inclusion: Some(ReceiptInclusionDeclarationV1 {
                owner_root_digest: owner_cut.root_digest.clone(),
                receipt_leaf_digest: receipt_digest,
                leaf_index: 0,
                tree_size: 1,
                proof_ref: "proof:declared-only".into(),
            }),
        }
    }

    fn root_for(owner_index: usize, sequence: u64) -> char {
        match (owner_index, sequence) {
            (0, 0) => 'a',
            (0, 1) => 'b',
            (0, 2) => 'c',
            (1, 0) => 'd',
            (1, 1) => 'e',
            (1, 2) => 'g',
            _ => panic!("bounded oracle root lookup out of range"),
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
                cut("owner:a", positions[0], root_for(0, positions[0])),
                cut("owner:b", positions[1], root_for(1, positions[1])),
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
            links.push(OwnerConsistencyLinkV1 {
                owner_ref: owner_ref.into(),
                history_ref: format!("history:{owner_ref}"),
                ancestor_root_digest: digest(root_for(owner_index, ancestor)),
                descendant_root_digest: digest(root_for(owner_index, descendant)),
                consistency_proof_ref: format!("proof:{owner_ref}:{ancestor}:{descendant}"),
            });
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
        let link = OwnerConsistencyLinkV1 {
            owner_ref: "owner:grants".into(),
            history_ref: first_cut.history_ref.clone(),
            ancestor_root_digest: first_cut.root_digest,
            descendant_root_digest: later_cut.root_digest,
            consistency_proof_ref: "proof:consistency:1".into(),
        };
        assert_eq!(first.compare(&later, &[link]), Ok(FrontierRelationV1::Before));
    }

    #[test]
    fn crossed_owner_cuts_are_concurrent_not_totally_ordered() {
        let a = frontier("frontier:a", vec![cut("owner:a", 10, 'a'), cut("owner:b", 20, 'b')]);
        let b = frontier("frontier:b", vec![cut("owner:a", 20, 'c'), cut("owner:b", 10, 'd')]);
        let links = vec![
            OwnerConsistencyLinkV1 {
                owner_ref: "owner:a".into(),
                history_ref: "history:owner:a".into(),
                ancestor_root_digest: digest('a'),
                descendant_root_digest: digest('c'),
                consistency_proof_ref: "proof:a".into(),
            },
            OwnerConsistencyLinkV1 {
                owner_ref: "owner:b".into(),
                history_ref: "history:owner:b".into(),
                ancestor_root_digest: digest('d'),
                descendant_root_digest: digest('b'),
                consistency_proof_ref: "proof:b".into(),
            },
        ];
        assert_eq!(a.compare(&b, &links), Ok(FrontierRelationV1::Concurrent));
    }

    #[test]
    fn equal_owner_cuts_with_different_aggregate_root_are_unknown() {
        let a = frontier("frontier:a", vec![cut("owner:a", 10, 'a')]);
        let mut b = frontier("frontier:b", vec![cut("owner:a", 10, 'a')]);
        b.checkpoint_root_digest = digest('e');
        b.owner_cuts[0].checkpoint_inclusion.checkpoint_root_digest = digest('e');
        assert_eq!(a.compare(&b, &[]), Ok(FrontierRelationV1::Unknown));
    }

    #[test]
    fn equal_sequence_with_different_root_is_a_fork_conflict() {
        let a = frontier("frontier:a", vec![cut("owner:a", 10, 'a')]);
        let b = frontier("frontier:b", vec![cut("owner:a", 10, 'b')]);
        assert_eq!(a.compare(&b, &[]), Ok(FrontierRelationV1::Conflict));
    }

    #[test]
    fn different_prefixes_without_consistency_proof_are_unknown() {
        let a = frontier("frontier:a", vec![cut("owner:a", 10, 'a')]);
        let b = frontier("frontier:b", vec![cut("owner:a", 20, 'b')]);
        assert_eq!(a.compare(&b, &[]), Ok(FrontierRelationV1::Unknown));
    }

    #[test]
    fn covered_receipt_requires_a_matching_inclusion_declaration() {
        let owner_cut = cut("owner:grants", 10, 'a');
        let frontier = frontier("frontier:1", vec![owner_cut.clone()]);
        let receipt = receipt(5, &owner_cut);
        assert_eq!(
            frontier.classify_receipt(&receipt),
            Ok(ReceiptVisibilityV1::DeclaredIncluded)
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
        let frontier = frontier("frontier:1", vec![owner_cut.clone()]);
        assert_eq!(
            frontier.classify_receipt(&receipt(11, &owner_cut)),
            Ok(ReceiptVisibilityV1::NotKnown)
        );
    }

    #[test]
    fn gaps_make_inclusion_and_absence_indeterminate() {
        let mut owner_cut = cut("owner:grants", 10, 'a');
        owner_cut.gaps.push(SequenceRangeV1 { first: 3, last: 4 });
        let frontier = frontier("frontier:1", vec![owner_cut.clone()]);
        assert_eq!(
            frontier.classify_receipt(&receipt(5, &owner_cut)),
            Ok(ReceiptVisibilityV1::Indeterminate)
        );
        assert!(!frontier.prefix_coverage_is_complete("owner:grants", 10));
    }

    #[test]
    fn a_frontier_does_not_accept_another_verifiers_receipt() {
        let owner_cut = cut("owner:grants", 10, 'a');
        let frontier = frontier("frontier:1", vec![owner_cut.clone()]);
        let mut receipt = receipt(5, &owner_cut);
        receipt.verifier_ref = "did:mycelix:verifier:other-v1".into();
        assert_eq!(
            frontier.classify_receipt(&receipt),
            Err(FrontierModelErrorV1::ForeignVerifier)
        );
    }

    #[test]
    fn uncovered_causal_dependency_is_indeterminate() {
        let owner_cut = cut("owner:grants", 10, 'a');
        let frontier = frontier("frontier:1", vec![owner_cut.clone()]);
        let mut receipt = receipt(5, &owner_cut);
        receipt.causal_dependencies.push(OwnerPositionV1 {
            owner_ref: "owner:payments".into(),
            sequence: 2,
        });
        assert_eq!(
            frontier.classify_receipt(&receipt),
            Ok(ReceiptVisibilityV1::Indeterminate)
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
        let link = OwnerConsistencyLinkV1 {
            owner_ref: "owner:a".into(),
            history_ref: "history:owner:a".into(),
            ancestor_root_digest: digest('a'),
            descendant_root_digest: digest('b'),
            consistency_proof_ref: "proof:consistency".into(),
        };
        assert_eq!(left.compare(&right, &[link]), Ok(FrontierRelationV1::Unknown));
    }

    #[test]
    fn same_owner_dependency_must_precede_its_receipt() {
        let owner_cut = cut("owner:a", 10, 'a');
        let mut candidate = receipt(5, &owner_cut);
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
    fn invalid_cut_cannot_claim_prefix_coverage() {
        let mut candidate = cut("owner:a", 10, 'a');
        candidate.gaps = vec![
            SequenceRangeV1 { first: 5, last: 4 },
        ];
        assert!(!candidate.prefix_is_complete_through(10));
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
