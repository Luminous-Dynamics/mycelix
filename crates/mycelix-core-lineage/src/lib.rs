#![no_std]

extern crate alloc;

use alloc::{string::String, vec::Vec};
use sha2::{Digest, Sha256};

pub const MAX_PROFILE_BYTES: usize = 256;
pub const MAX_TRANSITIONS: usize = 4096;
pub const ROOTED_LINEAGE_PROFILE: &str = "mycelix-core-lineage-v1-sha256-framed-semantic";
const ROOTED_LINEAGE_DOMAIN: &[u8] = b"mycelix/core-lineage/rooted-lineage/v1";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineageError {
    EmptyProfile,
    ProfileTooLong,
    ProfileLeadingOrTrailingSpace,
    ProfileControlByte,
    ZeroDigest,
    TooManyTransitions,
    DomainMismatch,
    GenerationOverflow,
    DiscontinuousGeneration,
    PredecessorNodeMismatch,
    PredecessorSourceDescriptorMismatch,
    EffectiveTimeRegression,
    TransitionIdentityCollision,
    ParallelTransitionConflict,
    ForkConflict,
    UnreachableTransition,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct ProfiledDigest32 {
    profile: String,
    digest: [u8; 32],
}

impl ProfiledDigest32 {
    pub fn try_new(profile: impl Into<String>, digest: [u8; 32]) -> Result<Self, LineageError> {
        let profile = profile.into();
        validate_profile(&profile)?;
        if digest == [0; 32] {
            return Err(LineageError::ZeroDigest);
        }
        Ok(Self { profile, digest })
    }

    pub fn profile(&self) -> &str {
        &self.profile
    }

    pub fn digest(&self) -> &[u8; 32] {
        &self.digest
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RootAnchorFacts {
    lineage_domain_identity: ProfiledDigest32,
    generation: u64,
    node_identity: ProfiledDigest32,
    effective_at_ms: u64,
    source_descriptor_identity: ProfiledDigest32,
}

impl RootAnchorFacts {
    pub fn new(
        lineage_domain_identity: ProfiledDigest32,
        generation: u64,
        node_identity: ProfiledDigest32,
        effective_at_ms: u64,
        source_descriptor_identity: ProfiledDigest32,
    ) -> Self {
        Self {
            lineage_domain_identity,
            generation,
            node_identity,
            effective_at_ms,
            source_descriptor_identity,
        }
    }

    pub fn lineage_domain_identity(&self) -> &ProfiledDigest32 {
        &self.lineage_domain_identity
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn node_identity(&self) -> &ProfiledDigest32 {
        &self.node_identity
    }

    pub fn effective_at_ms(&self) -> u64 {
        self.effective_at_ms
    }

    pub fn source_descriptor_identity(&self) -> &ProfiledDigest32 {
        &self.source_descriptor_identity
    }
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct TransitionFacts {
    lineage_domain_identity: ProfiledDigest32,
    predecessor_generation: u64,
    predecessor_node_identity: ProfiledDigest32,
    predecessor_source_descriptor_identity: ProfiledDigest32,
    successor_generation: u64,
    successor_node_identity: ProfiledDigest32,
    successor_source_descriptor_identity: ProfiledDigest32,
    transition_semantic_identity: ProfiledDigest32,
    effective_at_ms: u64,
}

impl TransitionFacts {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        lineage_domain_identity: ProfiledDigest32,
        predecessor_generation: u64,
        predecessor_node_identity: ProfiledDigest32,
        predecessor_source_descriptor_identity: ProfiledDigest32,
        successor_generation: u64,
        successor_node_identity: ProfiledDigest32,
        successor_source_descriptor_identity: ProfiledDigest32,
        transition_semantic_identity: ProfiledDigest32,
        effective_at_ms: u64,
    ) -> Self {
        Self {
            lineage_domain_identity,
            predecessor_generation,
            predecessor_node_identity,
            predecessor_source_descriptor_identity,
            successor_generation,
            successor_node_identity,
            successor_source_descriptor_identity,
            transition_semantic_identity,
            effective_at_ms,
        }
    }

    pub fn transition_semantic_identity(&self) -> &ProfiledDigest32 {
        &self.transition_semantic_identity
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProjectedRootedLineage {
    root: RootAnchorFacts,
    transitions: Vec<TransitionFacts>,
    endpoint_generation: u64,
    endpoint_node_identity: ProfiledDigest32,
    endpoint_source_descriptor_identity: ProfiledDigest32,
    stable_commitment: [u8; 32],
}

impl ProjectedRootedLineage {
    pub fn root(&self) -> &RootAnchorFacts {
        &self.root
    }

    pub fn transitions(&self) -> &[TransitionFacts] {
        &self.transitions
    }

    pub fn endpoint_generation(&self) -> u64 {
        self.endpoint_generation
    }

    pub fn endpoint_node_identity(&self) -> &ProfiledDigest32 {
        &self.endpoint_node_identity
    }

    pub fn endpoint_source_descriptor_identity(&self) -> &ProfiledDigest32 {
        &self.endpoint_source_descriptor_identity
    }

    pub fn stable_commitment(&self) -> &[u8; 32] {
        &self.stable_commitment
    }

    pub fn stable_commitment_profile(&self) -> &'static str {
        ROOTED_LINEAGE_PROFILE
    }

    pub const fn grants_currentness(&self) -> bool {
        false
    }

    pub const fn grants_effect_authority(&self) -> bool {
        false
    }
}

pub fn project_rooted_lineage(
    root: RootAnchorFacts,
    transitions: &[TransitionFacts],
) -> Result<ProjectedRootedLineage, LineageError> {
    if transitions.len() > MAX_TRANSITIONS {
        return Err(LineageError::TooManyTransitions);
    }

    for transition in transitions {
        if transition.lineage_domain_identity != root.lineage_domain_identity {
            return Err(LineageError::DomainMismatch);
        }
    }

    let mut canonical = transitions.to_vec();
    canonical.sort();
    canonical.dedup();

    for left in 0..canonical.len() {
        for right in (left + 1)..canonical.len() {
            if canonical[left].transition_semantic_identity
                == canonical[right].transition_semantic_identity
                && canonical[left] != canonical[right]
            {
                return Err(LineageError::TransitionIdentityCollision);
            }
        }
    }

    let mut consumed = alloc::vec![false; canonical.len()];
    let mut topology = Vec::with_capacity(canonical.len());
    let mut generation = root.generation;
    let mut node = root.node_identity.clone();
    let mut source = root.source_descriptor_identity.clone();
    let mut effective_at_ms = root.effective_at_ms;

    loop {
        let mut candidates = Vec::new();
        for (index, transition) in canonical.iter().enumerate() {
            if consumed[index] || transition.predecessor_generation != generation {
                continue;
            }
            if transition.predecessor_node_identity != node {
                return Err(LineageError::PredecessorNodeMismatch);
            }
            if transition.predecessor_source_descriptor_identity != source {
                return Err(LineageError::PredecessorSourceDescriptorMismatch);
            }
            candidates.push(index);
        }

        if candidates.is_empty() {
            break;
        }

        let first = &canonical[candidates[0]];
        for index in candidates.iter().skip(1) {
            let other = &canonical[*index];
            let same_successor = first.successor_generation == other.successor_generation
                && first.successor_node_identity == other.successor_node_identity
                && first.successor_source_descriptor_identity
                    == other.successor_source_descriptor_identity;
            if same_successor {
                return Err(LineageError::ParallelTransitionConflict);
            }
            return Err(LineageError::ForkConflict);
        }

        let expected_generation = generation
            .checked_add(1)
            .ok_or(LineageError::GenerationOverflow)?;
        if first.successor_generation != expected_generation {
            return Err(LineageError::DiscontinuousGeneration);
        }
        if first.effective_at_ms < effective_at_ms {
            return Err(LineageError::EffectiveTimeRegression);
        }

        let index = candidates[0];
        consumed[index] = true;
        topology.push(first.clone());
        generation = first.successor_generation;
        node = first.successor_node_identity.clone();
        source = first.successor_source_descriptor_identity.clone();
        effective_at_ms = first.effective_at_ms;
    }

    if consumed.iter().any(|consumed| !consumed) {
        if canonical.iter().enumerate().any(|(index, transition)| {
            !consumed[index] && transition.predecessor_generation > generation
        }) {
            return Err(LineageError::DiscontinuousGeneration);
        }
        return Err(LineageError::UnreachableTransition);
    }

    let stable_commitment = stable_lineage_commitment(&root, &topology);

    Ok(ProjectedRootedLineage {
        root,
        transitions: topology,
        endpoint_generation: generation,
        endpoint_node_identity: node,
        endpoint_source_descriptor_identity: source,
        stable_commitment,
    })
}

fn validate_profile(profile: &str) -> Result<(), LineageError> {
    let raw = profile.as_bytes();
    if raw.is_empty() {
        return Err(LineageError::EmptyProfile);
    }
    if raw.len() > MAX_PROFILE_BYTES {
        return Err(LineageError::ProfileTooLong);
    }
    if raw.first() == Some(&b' ') || raw.last() == Some(&b' ') {
        return Err(LineageError::ProfileLeadingOrTrailingSpace);
    }
    if raw.iter().any(|byte| *byte < 0x20 || *byte == 0x7f) {
        return Err(LineageError::ProfileControlByte);
    }
    Ok(())
}

fn frame(output: &mut Vec<u8>, raw: &[u8]) {
    output.extend_from_slice(&(raw.len() as u64).to_le_bytes());
    output.extend_from_slice(raw);
}

fn frame_text(output: &mut Vec<u8>, value: &str) {
    frame(output, value.as_bytes());
}

fn frame_u64(output: &mut Vec<u8>, value: u64) {
    frame(output, &value.to_le_bytes());
}

fn frame_profiled_digest(output: &mut Vec<u8>, value: &ProfiledDigest32) {
    frame_text(output, value.profile());
    frame(output, value.digest());
}

fn stable_lineage_commitment(root: &RootAnchorFacts, transitions: &[TransitionFacts]) -> [u8; 32] {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(ROOTED_LINEAGE_DOMAIN);
    frame_text(&mut bytes, ROOTED_LINEAGE_PROFILE);
    frame_profiled_digest(&mut bytes, &root.lineage_domain_identity);
    frame_u64(&mut bytes, root.generation);
    frame_profiled_digest(&mut bytes, &root.node_identity);
    frame_profiled_digest(&mut bytes, &root.source_descriptor_identity);
    frame_u64(&mut bytes, root.effective_at_ms);
    frame_u64(&mut bytes, transitions.len() as u64);

    for transition in transitions {
        frame_u64(&mut bytes, transition.predecessor_generation);
        frame_profiled_digest(&mut bytes, &transition.predecessor_node_identity);
        frame_profiled_digest(
            &mut bytes,
            &transition.predecessor_source_descriptor_identity,
        );
        frame_u64(&mut bytes, transition.successor_generation);
        frame_profiled_digest(&mut bytes, &transition.successor_node_identity);
        frame_profiled_digest(&mut bytes, &transition.successor_source_descriptor_identity);
        frame_profiled_digest(&mut bytes, &transition.transition_semantic_identity);
        frame_u64(&mut bytes, transition.effective_at_ms);
    }

    let digest = Sha256::digest(&bytes);
    let mut output = [0_u8; 32];
    output.copy_from_slice(&digest);
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::{string::String, vec};

    fn id(profile: &str, byte: u8) -> ProfiledDigest32 {
        ProfiledDigest32::try_new(profile, [byte; 32]).expect("valid fixture identity")
    }

    fn fixture() -> (RootAnchorFacts, Vec<TransitionFacts>) {
        let domain = id("example-lineage-domain-v1", 0x11);
        let node0 = id("example-node-v1", 0x22);
        let node1 = id("example-node-v1", 0x44);
        let node2 = id("example-node-v1", 0x66);
        let source0 = id("example-source-v1", 0x33);
        let source2 = id("example-source-v1", 0x77);

        let root = RootAnchorFacts::new(domain.clone(), 0, node0.clone(), 1_000, source0.clone());
        let first = TransitionFacts::new(
            domain.clone(),
            0,
            node0,
            source0.clone(),
            1,
            node1.clone(),
            source0.clone(),
            id("example-transition-v1", 0x55),
            2_000,
        );
        let second = TransitionFacts::new(
            domain,
            1,
            node1,
            source0,
            2,
            node2,
            source2,
            id("example-transition-v1", 0x88),
            3_000,
        );
        (root, vec![first, second])
    }

    fn hex(bytes: &[u8]) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut output = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            output.push(HEX[(byte >> 4) as usize] as char);
            output.push(HEX[(byte & 0x0f) as usize] as char);
        }
        output
    }

    #[test]
    fn golden_vector_is_stable() {
        let (root, transitions) = fixture();
        let projected = project_rooted_lineage(root, &transitions).expect("qualified structure");
        assert_eq!(projected.endpoint_generation(), 2);
        assert_eq!(
            hex(projected.stable_commitment()),
            "89a02c2002d5e95e2cb972d3726dd1be0b88b049f01351e4a31ac75872a7d5c0"
        );
        assert!(!projected.grants_currentness());
        assert!(!projected.grants_effect_authority());
    }

    #[test]
    fn input_permutation_does_not_change_identity() {
        let (root, mut transitions) = fixture();
        let expected = project_rooted_lineage(root.clone(), &transitions)
            .expect("original")
            .stable_commitment;
        transitions.reverse();
        let actual = project_rooted_lineage(root, &transitions)
            .expect("permuted")
            .stable_commitment;
        assert_eq!(actual, expected);
    }

    #[test]
    fn exact_semantic_duplicate_is_harmless() {
        let (root, mut transitions) = fixture();
        let expected = project_rooted_lineage(root.clone(), &transitions)
            .expect("original")
            .stable_commitment;
        transitions.push(transitions[0].clone());
        let actual = project_rooted_lineage(root, &transitions)
            .expect("duplicate")
            .stable_commitment;
        assert_eq!(actual, expected);
    }

    #[test]
    fn source_descriptor_rotation_is_structurally_allowed() {
        let (root, transitions) = fixture();
        let projected =
            project_rooted_lineage(root, &transitions).expect("rotation is domain-qualified");
        assert_eq!(
            projected.endpoint_source_descriptor_identity(),
            &id("example-source-v1", 0x77)
        );
    }

    #[test]
    fn parallel_transition_conflict_fails_closed() {
        let (root, mut transitions) = fixture();
        let first = transitions[0].clone();
        transitions.push(TransitionFacts::new(
            first.lineage_domain_identity.clone(),
            first.predecessor_generation,
            first.predecessor_node_identity.clone(),
            first.predecessor_source_descriptor_identity.clone(),
            first.successor_generation,
            first.successor_node_identity.clone(),
            first.successor_source_descriptor_identity.clone(),
            id("example-transition-v1", 0x99),
            first.effective_at_ms,
        ));
        assert_eq!(
            project_rooted_lineage(root, &transitions),
            Err(LineageError::ParallelTransitionConflict)
        );
    }

    #[test]
    fn fork_fails_closed() {
        let (root, mut transitions) = fixture();
        let first = transitions[0].clone();
        transitions.push(TransitionFacts::new(
            first.lineage_domain_identity.clone(),
            first.predecessor_generation,
            first.predecessor_node_identity.clone(),
            first.predecessor_source_descriptor_identity.clone(),
            first.successor_generation,
            id("example-node-v1", 0xaa),
            first.successor_source_descriptor_identity.clone(),
            id("example-transition-v1", 0xbb),
            first.effective_at_ms,
        ));
        assert_eq!(
            project_rooted_lineage(root, &transitions),
            Err(LineageError::ForkConflict)
        );
    }

    #[test]
    fn transition_identity_collision_fails_closed() {
        let (root, mut transitions) = fixture();
        let first = transitions[0].clone();
        transitions.push(TransitionFacts::new(
            first.lineage_domain_identity.clone(),
            first.predecessor_generation,
            first.predecessor_node_identity.clone(),
            first.predecessor_source_descriptor_identity.clone(),
            first.successor_generation,
            id("example-node-v1", 0xaa),
            first.successor_source_descriptor_identity.clone(),
            first.transition_semantic_identity.clone(),
            first.effective_at_ms,
        ));
        assert_eq!(
            project_rooted_lineage(root, &transitions),
            Err(LineageError::TransitionIdentityCollision)
        );
    }

    #[test]
    fn discontinuous_generation_fails_closed() {
        let (root, mut transitions) = fixture();
        transitions[0].successor_generation = 2;
        assert_eq!(
            project_rooted_lineage(root, &transitions),
            Err(LineageError::DiscontinuousGeneration)
        );
    }

    #[test]
    fn wrong_predecessor_node_fails_closed() {
        let (root, mut transitions) = fixture();
        transitions[0].predecessor_node_identity = id("example-node-v1", 0xee);
        assert_eq!(
            project_rooted_lineage(root, &transitions),
            Err(LineageError::PredecessorNodeMismatch)
        );
    }

    #[test]
    fn wrong_predecessor_source_descriptor_fails_closed() {
        let (root, mut transitions) = fixture();
        transitions[0].predecessor_source_descriptor_identity = id("example-source-v1", 0xee);
        assert_eq!(
            project_rooted_lineage(root, &transitions),
            Err(LineageError::PredecessorSourceDescriptorMismatch)
        );
    }

    #[test]
    fn wrong_domain_fails_closed() {
        let (root, mut transitions) = fixture();
        transitions[0].lineage_domain_identity = id("other-domain-v1", 0xee);
        assert_eq!(
            project_rooted_lineage(root, &transitions),
            Err(LineageError::DomainMismatch)
        );
    }

    #[test]
    fn effective_time_regression_fails_closed() {
        let (root, mut transitions) = fixture();
        transitions[1].effective_at_ms = 1_500;
        assert_eq!(
            project_rooted_lineage(root, &transitions),
            Err(LineageError::EffectiveTimeRegression)
        );
    }

    #[test]
    fn unreachable_skipped_transition_fails_closed() {
        let (root, transitions) = fixture();
        assert_eq!(
            project_rooted_lineage(root, &transitions[1..]),
            Err(LineageError::DiscontinuousGeneration)
        );
    }

    #[test]
    fn root_only_is_valid_structure_but_not_currentness() {
        let (root, _) = fixture();
        let projected = project_rooted_lineage(root, &[]).expect("root-only structure");
        assert_eq!(projected.endpoint_generation(), 0);
        assert!(!projected.grants_currentness());
    }

    #[test]
    fn malformed_profile_and_zero_digest_are_rejected() {
        assert_eq!(
            ProfiledDigest32::try_new("", [1; 32]),
            Err(LineageError::EmptyProfile)
        );
        assert_eq!(
            ProfiledDigest32::try_new(" bad", [1; 32]),
            Err(LineageError::ProfileLeadingOrTrailingSpace)
        );
        assert_eq!(
            ProfiledDigest32::try_new("bad\nprofile", [1; 32]),
            Err(LineageError::ProfileControlByte)
        );
        assert_eq!(
            ProfiledDigest32::try_new("valid-profile", [0; 32]),
            Err(LineageError::ZeroDigest)
        );
    }

    #[test]
    fn resource_bound_is_checked_before_canonicalization() {
        let (root, transitions) = fixture();
        let excessive = alloc::vec![transitions[0].clone(); MAX_TRANSITIONS + 1];
        assert_eq!(
            project_rooted_lineage(root, &excessive),
            Err(LineageError::TooManyTransitions)
        );
    }
}
