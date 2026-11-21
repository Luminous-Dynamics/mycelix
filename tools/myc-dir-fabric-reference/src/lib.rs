//! Pure-Rust reference semantics for Mycelix Directory Fabric.
//!
//! This crate is deliberately not an authorization policy engine. It models
//! three bounded preconditions around an externally verified authority model:
//! causal-frontier freshness, narrow offline-lease validity, and bounded group
//! graph expansion. Full-context capability applicability and policy
//! subsumption remain owned by the existing Mycelix authority lanes.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const MAX_IDENTIFIER_BYTES: usize = 512;
pub type Digest = [u8; 32];

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum IdentifierError {
    Empty,
    TooLong { actual: usize, maximum: usize },
}

impl fmt::Display for IdentifierError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "identifier must not be empty or whitespace-only"),
            Self::TooLong { actual, maximum } => {
                write!(f, "identifier is {actual} bytes; maximum is {maximum}")
            }
        }
    }
}

impl std::error::Error for IdentifierError {}

macro_rules! string_id {
    ($name:ident) => {
        #[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, IdentifierError> {
                let value = value.into();
                if value.trim().is_empty() {
                    return Err(IdentifierError::Empty);
                }
                if value.len() > MAX_IDENTIFIER_BYTES {
                    return Err(IdentifierError::TooLong {
                        actual: value.len(),
                        maximum: MAX_IDENTIFIER_BYTES,
                    });
                }
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

string_id!(RealmId);
string_id!(AuthorityId);
string_id!(PrincipalId);
string_id!(ResourceId);
string_id!(ActionId);
string_id!(AudienceId);
string_id!(PurposeId);
string_id!(GroupId);

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct AuthorityPosition {
    authority: AuthorityId,
    sequence: u64,
    head_digest: Digest,
}

impl AuthorityPosition {
    pub fn new(
        authority: AuthorityId,
        sequence: u64,
        head_digest: Digest,
    ) -> Result<Self, FrontierError> {
        if sequence == 0 && head_digest != [0; 32] {
            return Err(FrontierError::InvalidGenesisDigest);
        }
        Ok(Self {
            authority,
            sequence,
            head_digest,
        })
    }

    pub fn authority(&self) -> &AuthorityId {
        &self.authority
    }

    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    pub fn head_digest(&self) -> &Digest {
        &self.head_digest
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FrontierError {
    DuplicateAuthority,
    DuplicateCoveragePosition,
    CoverageForUnknownAuthority,
    CoverageNotOlderThanHead,
    InvalidGenesisDigest,
    TooManyComponents,
    IdentifierTooLong,
}

impl fmt::Display for FrontierError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for FrontierError {}

fn append_identifier(bytes: &mut Vec<u8>, value: &str) -> Result<(), FrontierError> {
    let identifier = value.as_bytes();
    let length =
        u16::try_from(identifier.len()).map_err(|_| FrontierError::IdentifierTooLong)?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(identifier);
    Ok(())
}

fn append_position(
    bytes: &mut Vec<u8>,
    position: &AuthorityPosition,
) -> Result<(), FrontierError> {
    append_identifier(bytes, position.authority.as_str())?;
    bytes.extend_from_slice(&position.sequence.to_be_bytes());
    bytes.extend_from_slice(&position.head_digest);
    Ok(())
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorityFrontier {
    components: Vec<AuthorityPosition>,
}

impl AuthorityFrontier {
    /// Creates a canonical frontier by sorting components and rejecting
    /// duplicate authority identifiers. An empty frontier is the genesis
    /// lower bound; callers must not interpret it as proof of global freshness.
    pub fn new(mut components: Vec<AuthorityPosition>) -> Result<Self, FrontierError> {
        components.sort_by(|left, right| left.authority.cmp(&right.authority));
        for pair in components.windows(2) {
            if pair[0].authority == pair[1].authority {
                return Err(FrontierError::DuplicateAuthority);
            }
        }
        Ok(Self { components })
    }

    pub fn components(&self) -> &[AuthorityPosition] {
        &self.components
    }

    fn position(&self, authority: &AuthorityId) -> Option<&AuthorityPosition> {
        self.components
            .binary_search_by(|component| component.authority.cmp(authority))
            .ok()
            .map(|index| &self.components[index])
    }

    /// Canonical, domain-separated encoding for external digest/signature
    /// systems. This method does not hash or authenticate the returned bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, FrontierError> {
        let count = u32::try_from(self.components.len())
            .map_err(|_| FrontierError::TooManyComponents)?;
        let mut bytes = b"MYC-DIR-AUTHORITY-FRONTIER\0\x01".to_vec();
        bytes.extend_from_slice(&count.to_be_bytes());

        for component in &self.components {
            append_position(&mut bytes, component)?;
        }

        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FrontierRelation {
    Equal,
    Dominates,
    Behind,
    Incomparable,
}

/// A snapshot's frontier plus exact older positions that an independent
/// verifier has already established as covered by the snapshot's authority
/// histories. This crate checks exact binding and shape only; it does not
/// authenticate signatures or validate hash-chain/inclusion proofs.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FrontierEvidence {
    frontier: AuthorityFrontier,
    covered_positions: Vec<AuthorityPosition>,
}

impl FrontierEvidence {
    /// Canonical encoding binds both the frontier heads and the exact ancestry
    /// positions supplied to the independent proof verifier.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, FrontierError> {
        let frontier_bytes = self.frontier.canonical_bytes()?;
        let frontier_length = u32::try_from(frontier_bytes.len())
            .map_err(|_| FrontierError::TooManyComponents)?;
        let count = u32::try_from(self.covered_positions.len())
            .map_err(|_| FrontierError::TooManyComponents)?;
        let mut bytes = b"MYC-DIR-FRONTIER-EVIDENCE\0\x01".to_vec();
        bytes.extend_from_slice(&frontier_length.to_be_bytes());
        bytes.extend_from_slice(&frontier_bytes);
        bytes.extend_from_slice(&count.to_be_bytes());
        for position in &self.covered_positions {
            append_position(&mut bytes, position)?;
        }
        Ok(bytes)
    }

    pub fn new(
        frontier: AuthorityFrontier,
        mut covered_positions: Vec<AuthorityPosition>,
    ) -> Result<Self, FrontierError> {
        covered_positions.sort();

        for pair in covered_positions.windows(2) {
            if pair[0].authority == pair[1].authority
                && pair[0].sequence == pair[1].sequence
            {
                return Err(FrontierError::DuplicateCoveragePosition);
            }
        }

        for covered in &covered_positions {
            match frontier.position(&covered.authority) {
                Some(head) if head.sequence > covered.sequence => {}
                Some(_) => return Err(FrontierError::CoverageNotOlderThanHead),
                None => return Err(FrontierError::CoverageForUnknownAuthority),
            }
        }

        Ok(Self {
            frontier,
            covered_positions,
        })
    }

    pub fn frontier(&self) -> &AuthorityFrontier {
        &self.frontier
    }

    /// Determines whether this local snapshot covers a caller's minimum
    /// frontier. Sequence numbers alone are insufficient: every older
    /// position requires an exact ancestry witness, and same-sequence forks
    /// are incomparable.
    pub fn relation_to(&self, required: &AuthorityFrontier) -> FrontierRelation {
        let mut behind = false;
        let mut incomparable = false;
        let mut strictly_ahead = false;

        for required_position in required.components() {
            match self.frontier.position(&required_position.authority) {
                None if required_position.sequence == 0 => {}
                None => behind = true,
                Some(current) => match current.sequence.cmp(&required_position.sequence) {
                    std::cmp::Ordering::Less => behind = true,
                    std::cmp::Ordering::Equal => {
                        if current.head_digest != required_position.head_digest {
                            incomparable = true;
                        }
                    }
                    std::cmp::Ordering::Greater => {
                        if self.covered_positions.iter().any(|covered| {
                            covered == required_position
                        }) {
                            strictly_ahead = true;
                        } else {
                            incomparable = true;
                        }
                    }
                },
            }
        }

        // Extra, non-genesis authority components are progress relative to a
        // lower-bound vector. They do not repair a missing/behind required
        // component; mixed progress is therefore incomparable.
        for current in self.frontier.components() {
            if current.sequence > 0 && required.position(&current.authority).is_none() {
                strictly_ahead = true;
            }
        }

        if incomparable || (behind && strictly_ahead) {
            FrontierRelation::Incomparable
        } else if behind {
            FrontierRelation::Behind
        } else if strictly_ahead {
            FrontierRelation::Dominates
        } else {
            FrontierRelation::Equal
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PolicySnapshot {
    realm: RealmId,
    policy_epoch: u64,
    persisted_high_water_mark: u64,
    policy_digest: Digest,
    frontier_evidence: FrontierEvidence,
}

impl PolicySnapshot {
    pub fn new(
        realm: RealmId,
        policy_epoch: u64,
        persisted_high_water_mark: u64,
        policy_digest: Digest,
        frontier_evidence: FrontierEvidence,
    ) -> Self {
        Self {
            realm,
            policy_epoch,
            persisted_high_water_mark,
            policy_digest,
            frontier_evidence,
        }
    }

    pub fn realm(&self) -> &RealmId {
        &self.realm
    }

    pub fn policy_epoch(&self) -> u64 {
        self.policy_epoch
    }

    pub fn policy_digest(&self) -> &Digest {
        &self.policy_digest
    }

    /// Canonical encoding of the remote policy snapshot. The persisted local
    /// high-water mark is intentionally excluded: it is local anti-rollback
    /// state, not a field asserted by the snapshot issuer.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, FrontierError> {
        let evidence = self.frontier_evidence.canonical_bytes()?;
        let evidence_length =
            u32::try_from(evidence.len()).map_err(|_| FrontierError::TooManyComponents)?;
        let mut bytes = b"MYC-DIR-POLICY-SNAPSHOT\0\x01".to_vec();
        append_identifier(&mut bytes, self.realm.as_str())?;
        bytes.extend_from_slice(&self.policy_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.policy_digest);
        bytes.extend_from_slice(&evidence_length.to_be_bytes());
        bytes.extend_from_slice(&evidence);
        Ok(bytes)
    }

    pub fn frontier_evidence(&self) -> &FrontierEvidence {
        &self.frontier_evidence
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SnapshotAssessment {
    /// The snapshot covers the caller-supplied lower bound and is not older
    /// than the persisted local policy high-water mark. This is a precondition,
    /// never an authorization decision or a claim of global freshness.
    SatisfiesRequiredFrontier,
    BehindRequiredFrontier,
    IncomparableRequiredFrontier,
    RollbackDetected,
    RealmMismatch,
}

pub fn assess_snapshot(
    snapshot: &PolicySnapshot,
    expected_realm: &RealmId,
    required_frontier: &AuthorityFrontier,
) -> SnapshotAssessment {
    if &snapshot.realm != expected_realm {
        return SnapshotAssessment::RealmMismatch;
    }

    if snapshot.policy_epoch < snapshot.persisted_high_water_mark {
        return SnapshotAssessment::RollbackDetected;
    }

    match snapshot.frontier_evidence.relation_to(required_frontier) {
        FrontierRelation::Equal | FrontierRelation::Dominates => {
            SnapshotAssessment::SatisfiesRequiredFrontier
        }
        FrontierRelation::Behind => SnapshotAssessment::BehindRequiredFrontier,
        FrontierRelation::Incomparable => SnapshotAssessment::IncomparableRequiredFrontier,
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AccessScope {
    pub realm: RealmId,
    pub principal: PrincipalId,
    pub resource: ResourceId,
    pub action: ActionId,
    pub audience: AudienceId,
    pub purpose: PurposeId,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OperationRisk {
    LowRiskReadOnly,
    Elevated,
    PrivilegedOrWrite,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ClockWindow {
    earliest_ms: u64,
    latest_ms: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClockError {
    InvertedInterval,
}

impl ClockWindow {
    pub fn new(earliest_ms: u64, latest_ms: u64) -> Result<Self, ClockError> {
        if earliest_ms > latest_ms {
            return Err(ClockError::InvertedInterval);
        }
        Ok(Self {
            earliest_ms,
            latest_ms,
        })
    }

    pub fn uncertainty_ms(&self) -> u64 {
        self.latest_ms - self.earliest_ms
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OfflineLease {
    scope: AccessScope,
    policy_epoch: u64,
    minimum_frontier: AuthorityFrontier,
    policy_digest: Digest,
    issued_at_ms: u64,
    not_before_ms: u64,
    expires_at_ms: u64,
    max_clock_uncertainty_ms: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaseError {
    InvalidTimeRange,
}

impl OfflineLease {
    pub fn new(
        scope: AccessScope,
        policy_epoch: u64,
        minimum_frontier: AuthorityFrontier,
        policy_digest: Digest,
        issued_at_ms: u64,
        not_before_ms: u64,
        expires_at_ms: u64,
        max_clock_uncertainty_ms: u64,
    ) -> Result<Self, LeaseError> {
        if issued_at_ms > not_before_ms || not_before_ms >= expires_at_ms {
            return Err(LeaseError::InvalidTimeRange);
        }
        Ok(Self {
            scope,
            policy_epoch,
            minimum_frontier,
            policy_digest,
            issued_at_ms,
            not_before_ms,
            expires_at_ms,
            max_clock_uncertainty_ms,
        })
    }

    /// Canonical bytes for signing or digesting the complete lease contract.
    /// This does not create or verify a signature.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, FrontierError> {
        let frontier = self.minimum_frontier.canonical_bytes()?;
        let frontier_length =
            u32::try_from(frontier.len()).map_err(|_| FrontierError::TooManyComponents)?;
        let mut bytes = b"MYC-DIR-OFFLINE-LEASE\0\x01".to_vec();
        append_identifier(&mut bytes, self.scope.realm.as_str())?;
        append_identifier(&mut bytes, self.scope.principal.as_str())?;
        append_identifier(&mut bytes, self.scope.resource.as_str())?;
        append_identifier(&mut bytes, self.scope.action.as_str())?;
        append_identifier(&mut bytes, self.scope.audience.as_str())?;
        append_identifier(&mut bytes, self.scope.purpose.as_str())?;
        bytes.extend_from_slice(&self.policy_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.policy_digest);
        bytes.extend_from_slice(&frontier_length.to_be_bytes());
        bytes.extend_from_slice(&frontier);
        bytes.extend_from_slice(&self.issued_at_ms.to_be_bytes());
        bytes.extend_from_slice(&self.not_before_ms.to_be_bytes());
        bytes.extend_from_slice(&self.expires_at_ms.to_be_bytes());
        bytes.extend_from_slice(&self.max_clock_uncertainty_ms.to_be_bytes());
        Ok(bytes)
    }

    pub fn expires_at_ms(&self) -> u64 {
        self.expires_at_ms
    }

    pub fn issued_at_ms(&self) -> u64 {
        self.issued_at_ms
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaseAssessment {
    ValidForBoundedOfflineRead,
    ScopeMismatch,
    SnapshotRealmMismatch,
    SnapshotRollbackDetected,
    PolicyEpochMismatch,
    PolicyDigestMismatch,
    BehindLeaseFrontier,
    IncomparableLeaseFrontier,
    RiskNotEligibleForOfflineLease,
    ClockUncertaintyTooWide,
    NotYetValid,
    Expired,
    TimeBoundaryAmbiguous,
}

/// Checks only whether an offline lease is valid for one exact low-risk read.
/// The caller must separately require SatisfiesRequiredFrontier and run the
/// authoritative #4743/#4854 authorization evaluation before any effect.
pub fn assess_offline_lease(
    lease: &OfflineLease,
    requested_scope: &AccessScope,
    snapshot: &PolicySnapshot,
    operation_risk: OperationRisk,
    now: ClockWindow,
) -> LeaseAssessment {
    if &snapshot.realm != &requested_scope.realm {
        return LeaseAssessment::SnapshotRealmMismatch;
    }
    if &lease.scope != requested_scope {
        return LeaseAssessment::ScopeMismatch;
    }
    if snapshot.policy_epoch < snapshot.persisted_high_water_mark {
        return LeaseAssessment::SnapshotRollbackDetected;
    }
    if lease.policy_epoch != snapshot.policy_epoch {
        return LeaseAssessment::PolicyEpochMismatch;
    }
    if &lease.policy_digest != snapshot.policy_digest() {
        return LeaseAssessment::PolicyDigestMismatch;
    }
    match snapshot
        .frontier_evidence
        .relation_to(&lease.minimum_frontier)
    {
        FrontierRelation::Equal | FrontierRelation::Dominates => {}
        FrontierRelation::Behind => return LeaseAssessment::BehindLeaseFrontier,
        FrontierRelation::Incomparable => return LeaseAssessment::IncomparableLeaseFrontier,
    }
    if operation_risk != OperationRisk::LowRiskReadOnly {
        return LeaseAssessment::RiskNotEligibleForOfflineLease;
    }
    if now.uncertainty_ms() > lease.max_clock_uncertainty_ms {
        return LeaseAssessment::ClockUncertaintyTooWide;
    }
    if now.latest_ms < lease.not_before_ms {
        return LeaseAssessment::NotYetValid;
    }
    if now.earliest_ms >= lease.expires_at_ms {
        return LeaseAssessment::Expired;
    }
    if now.earliest_ms < lease.not_before_ms || now.latest_ms >= lease.expires_at_ms {
        return LeaseAssessment::TimeBoundaryAmbiguous;
    }
    LeaseAssessment::ValidForBoundedOfflineRead
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GroupEdge {
    /// A member group inherits the permissions of its parent group.
    pub child: GroupId,
    pub parent: GroupId,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExpansionLimits {
    pub max_depth: usize,
    pub max_nodes: usize,
    pub max_edges: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GroupExpansionError {
    CycleDetected,
    DepthLimitExceeded,
    NodeLimitExceeded,
    EdgeLimitExceeded,
}

#[derive(Clone, Debug)]
struct DfsFrame {
    group: GroupId,
    next_parent: usize,
    depth: usize,
}

/// Expands direct group memberships through child -> parent inheritance.
/// Cycles or budget exhaustion reject the whole expansion; no truncated,
/// partially successful group set is returned.
pub fn expand_group_ancestors(
    direct_groups: &[GroupId],
    edges: &[GroupEdge],
    limits: ExpansionLimits,
) -> Result<BTreeSet<GroupId>, GroupExpansionError> {
    if edges.len() > limits.max_edges {
        return Err(GroupExpansionError::EdgeLimitExceeded);
    }

    let mut adjacency: BTreeMap<GroupId, Vec<GroupId>> = BTreeMap::new();
    for edge in edges {
        adjacency
            .entry(edge.child.clone())
            .or_default()
            .push(edge.parent.clone());
    }
    for parents in adjacency.values_mut() {
        parents.sort();
        parents.dedup();
    }

    let roots: BTreeSet<GroupId> = direct_groups.iter().cloned().collect();
    if roots.len() > limits.max_nodes {
        return Err(GroupExpansionError::NodeLimitExceeded);
    }

    let mut completed = BTreeSet::new();

    for root in roots {
        if completed.contains(&root) {
            continue;
        }
        if limits.max_nodes == 0 {
            return Err(GroupExpansionError::NodeLimitExceeded);
        }

        let mut active = BTreeSet::new();
        active.insert(root.clone());
        let mut stack = vec![DfsFrame {
            group: root,
            next_parent: 0,
            depth: 0,
        }];

        while !stack.is_empty() {
            let next = {
                let frame = stack.last_mut().expect("stack is checked as non-empty");
                let parent = adjacency
                    .get(&frame.group)
                    .and_then(|parents| parents.get(frame.next_parent))
                    .cloned();
                if parent.is_some() {
                    frame.next_parent += 1;
                }
                parent.map(|parent| (parent, frame.depth))
            };

            match next {
                Some((parent, parent_depth)) => {
                    if active.contains(&parent) {
                        return Err(GroupExpansionError::CycleDetected);
                    }
                    if completed.contains(&parent) {
                        continue;
                    }

                    let depth = parent_depth + 1;
                    if depth > limits.max_depth {
                        return Err(GroupExpansionError::DepthLimitExceeded);
                    }
                    if completed.len() + active.len() >= limits.max_nodes {
                        return Err(GroupExpansionError::NodeLimitExceeded);
                    }

                    active.insert(parent.clone());
                    stack.push(DfsFrame {
                        group: parent,
                        next_parent: 0,
                        depth,
                    });
                }
                None => {
                    let frame = stack.pop().expect("stack is checked as non-empty");
                    active.remove(&frame.group);
                    completed.insert(frame.group);
                }
            }
        }
    }

    Ok(completed)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identifier<T>(
        value: &str,
        constructor: impl FnOnce(String) -> Result<T, IdentifierError>,
    ) -> T {
        constructor(value.to_owned()).expect("valid test identifier")
    }

    fn realm(value: &str) -> RealmId {
        identifier(value, RealmId::new)
    }

    fn authority(value: &str) -> AuthorityId {
        identifier(value, AuthorityId::new)
    }

    fn principal(value: &str) -> PrincipalId {
        identifier(value, PrincipalId::new)
    }

    fn resource(value: &str) -> ResourceId {
        identifier(value, ResourceId::new)
    }

    fn action(value: &str) -> ActionId {
        identifier(value, ActionId::new)
    }

    fn audience(value: &str) -> AudienceId {
        identifier(value, AudienceId::new)
    }

    fn purpose(value: &str) -> PurposeId {
        identifier(value, PurposeId::new)
    }

    fn group(value: &str) -> GroupId {
        identifier(value, GroupId::new)
    }

    fn position(name: &str, sequence: u64, marker: u8) -> AuthorityPosition {
        let mut digest = [0; 32];
        if sequence > 0 {
            digest[0] = marker;
        }
        AuthorityPosition::new(authority(name), sequence, digest).expect("valid position")
    }

    fn frontier(positions: Vec<AuthorityPosition>) -> AuthorityFrontier {
        AuthorityFrontier::new(positions).expect("valid frontier")
    }

    fn scope() -> AccessScope {
        AccessScope {
            realm: realm("realm-a"),
            principal: principal("person-1"),
            resource: resource("record-7"),
            action: action("read"),
            audience: audience("service-a"),
            purpose: purpose("case-review"),
        }
    }

    fn policy_snapshot(evidence: FrontierEvidence) -> PolicySnapshot {
        PolicySnapshot::new(realm("realm-a"), 12, 12, [7; 32], evidence)
    }

    #[test]
    fn frontier_encoding_is_canonical_independent_of_input_order() {
        let left = frontier(vec![position("issuer-b", 2, 2), position("realm-root", 4, 4)]);
        let right = frontier(vec![position("realm-root", 4, 4), position("issuer-b", 2, 2)]);
        assert_eq!(left.canonical_bytes().unwrap(), right.canonical_bytes().unwrap());
    }

    #[test]
    fn duplicate_authority_components_are_rejected() {
        let result = AuthorityFrontier::new(vec![
            position("issuer-a", 1, 1),
            position("issuer-a", 2, 2),
        ]);
        assert_eq!(result, Err(FrontierError::DuplicateAuthority));
    }

    #[test]
    fn higher_sequence_without_ancestry_is_incomparable_not_fresh() {
        let current = frontier(vec![position("issuer-a", 4, 4)]);
        let evidence = FrontierEvidence::new(current, vec![]).unwrap();
        let required = frontier(vec![position("issuer-a", 3, 3)]);
        assert_eq!(evidence.relation_to(&required), FrontierRelation::Incomparable);
    }

    #[test]
    fn exact_ancestry_witness_allows_frontier_dominance() {
        let old = position("issuer-a", 3, 3);
        let current = frontier(vec![position("issuer-a", 4, 4)]);
        let evidence = FrontierEvidence::new(current, vec![old.clone()]).unwrap();
        let required = frontier(vec![old]);
        assert_eq!(evidence.relation_to(&required), FrontierRelation::Dominates);
    }

    #[test]
    fn same_sequence_different_digest_is_a_fork() {
        let current = frontier(vec![position("issuer-a", 5, 8)]);
        let evidence = FrontierEvidence::new(current, vec![]).unwrap();
        let required = frontier(vec![position("issuer-a", 5, 9)]);
        assert_eq!(evidence.relation_to(&required), FrontierRelation::Incomparable);
    }

    #[test]
    fn mixed_behind_and_independent_progress_is_incomparable() {
        let current = frontier(vec![
            position("issuer-a", 2, 2),
            position("issuer-b", 1, 1),
        ]);
        let evidence = FrontierEvidence::new(current, vec![]).unwrap();
        let required = frontier(vec![position("issuer-a", 3, 3)]);
        assert_eq!(evidence.relation_to(&required), FrontierRelation::Incomparable);
    }

    #[test]
    fn snapshot_rejects_realm_mismatch_and_policy_rollback() {
        let empty = FrontierEvidence::new(frontier(vec![]), vec![]).unwrap();
        let wrong_realm = PolicySnapshot::new(realm("realm-b"), 12, 12, [7; 32], empty.clone());
        assert_eq!(
            assess_snapshot(&wrong_realm, &realm("realm-a"), &frontier(vec![])),
            SnapshotAssessment::RealmMismatch
        );

        let rollback = PolicySnapshot::new(realm("realm-a"), 11, 12, [7; 32], empty);
        assert_eq!(
            assess_snapshot(&rollback, &realm("realm-a"), &frontier(vec![])),
            SnapshotAssessment::RollbackDetected
        );
    }

    #[test]
    fn snapshot_satisfies_only_an_explicitly_covered_minimum_frontier() {
        let old = position("issuer-a", 3, 3);
        let evidence = FrontierEvidence::new(
            frontier(vec![position("issuer-a", 4, 4)]),
            vec![old.clone()],
        )
        .unwrap();
        let snapshot = policy_snapshot(evidence);
        assert_eq!(
            assess_snapshot(&snapshot, &realm("realm-a"), &frontier(vec![old])),
            SnapshotAssessment::SatisfiesRequiredFrontier
        );
    }

    #[test]
    fn offline_lease_is_exact_scope_policy_and_low_risk_bound() {
        let requested = scope();
        let empty = FrontierEvidence::new(frontier(vec![]), vec![]).unwrap();
        let snapshot = policy_snapshot(empty);
        let lease = OfflineLease::new(requested.clone(), 12, frontier(vec![]), [7; 32], 100, 100, 200, 5).unwrap();
        let now = ClockWindow::new(150, 152).unwrap();

        assert_eq!(
            assess_offline_lease(
                &lease,
                &requested,
                &snapshot,
                OperationRisk::LowRiskReadOnly,
                now
            ),
            LeaseAssessment::ValidForBoundedOfflineRead
        );
        assert_eq!(
            assess_offline_lease(
                &lease,
                &requested,
                &snapshot,
                OperationRisk::PrivilegedOrWrite,
                now
            ),
            LeaseAssessment::RiskNotEligibleForOfflineLease
        );

        let mut other_scope = requested.clone();
        other_scope.audience = audience("service-b");
        assert_eq!(
            assess_offline_lease(
                &lease,
                &other_scope,
                &snapshot,
                OperationRisk::LowRiskReadOnly,
                now
            ),
            LeaseAssessment::ScopeMismatch
        );
    }

    #[test]
    fn offline_lease_requires_its_pinned_policy_epoch_and_frontier() {
        let requested = scope();
        let old = position("issuer-a", 3, 3);
        let lease = OfflineLease::new(
            requested.clone(),
            12,
            frontier(vec![old.clone()]),
            [7; 32],
            100,
            100,
            200,
            5,
        )
        .unwrap();

        let behind = FrontierEvidence::new(frontier(vec![position("issuer-a", 2, 2)]), vec![])
            .unwrap();
        let snapshot = policy_snapshot(behind);
        assert_eq!(
            assess_offline_lease(
                &lease,
                &requested,
                &snapshot,
                OperationRisk::LowRiskReadOnly,
                ClockWindow::new(150, 152).unwrap()
            ),
            LeaseAssessment::BehindLeaseFrontier
        );

        let covered = FrontierEvidence::new(
            frontier(vec![position("issuer-a", 4, 4)]),
            vec![old],
        )
        .unwrap();
        let wrong_epoch = PolicySnapshot::new(realm("realm-a"), 13, 13, [7; 32], covered);
        assert_eq!(
            assess_offline_lease(
                &lease,
                &requested,
                &wrong_epoch,
                OperationRisk::LowRiskReadOnly,
                ClockWindow::new(150, 152).unwrap()
            ),
            LeaseAssessment::PolicyEpochMismatch
        );
    }

    #[test]
    fn canonical_evidence_encoding_binds_ancestry_witnesses() {
        let head = frontier(vec![position("issuer-a", 4, 4)]);
        let without = FrontierEvidence::new(head.clone(), vec![]).unwrap();
        let with = FrontierEvidence::new(head, vec![position("issuer-a", 3, 3)]).unwrap();
        assert_ne!(without.canonical_bytes().unwrap(), with.canonical_bytes().unwrap());
    }

    #[test]
    fn clock_uncertainty_and_expiry_boundary_never_extend_a_lease() {
        let requested = scope();
        let evidence = FrontierEvidence::new(frontier(vec![]), vec![]).unwrap();
        let snapshot = policy_snapshot(evidence);
        let lease = OfflineLease::new(requested.clone(), 12, frontier(vec![]), [7; 32], 100, 100, 200, 5).unwrap();

        let too_uncertain = ClockWindow::new(150, 160).unwrap();
        assert_eq!(
            assess_offline_lease(
                &lease,
                &requested,
                &snapshot,
                OperationRisk::LowRiskReadOnly,
                too_uncertain
            ),
            LeaseAssessment::ClockUncertaintyTooWide
        );

        let straddles_expiry = ClockWindow::new(198, 202).unwrap();
        assert_eq!(
            assess_offline_lease(
                &lease,
                &requested,
                &snapshot,
                OperationRisk::LowRiskReadOnly,
                straddles_expiry
            ),
            LeaseAssessment::TimeBoundaryAmbiguous
        );

        let exact_after_expiry = ClockWindow::new(200, 200).unwrap();
        assert_eq!(
            assess_offline_lease(
                &lease,
                &requested,
                &snapshot,
                OperationRisk::LowRiskReadOnly,
                exact_after_expiry
            ),
            LeaseAssessment::Expired
        );
    }

    #[test]
    fn group_expansion_returns_transitive_ancestors() {
        let result = expand_group_ancestors(
            &[group("engineering")],
            &[
                GroupEdge {
                    child: group("engineering"),
                    parent: group("staff"),
                },
                GroupEdge {
                    child: group("staff"),
                    parent: group("all-employees"),
                },
            ],
            ExpansionLimits {
                max_depth: 2,
                max_nodes: 3,
                max_edges: 2,
            },
        )
        .unwrap();

        assert_eq!(
            result,
            BTreeSet::from([
                group("engineering"),
                group("staff"),
                group("all-employees")
            ])
        );
    }

    #[test]
    fn group_cycle_is_rejected_without_partial_expansion() {
        let result = expand_group_ancestors(
            &[group("a")],
            &[
                GroupEdge {
                    child: group("a"),
                    parent: group("b"),
                },
                GroupEdge {
                    child: group("b"),
                    parent: group("a"),
                },
            ],
            ExpansionLimits {
                max_depth: 8,
                max_nodes: 8,
                max_edges: 8,
            },
        );

        assert_eq!(result, Err(GroupExpansionError::CycleDetected));
    }

    #[test]
    fn group_budget_exhaustion_is_not_silently_truncated() {
        let result = expand_group_ancestors(
            &[group("a")],
            &[
                GroupEdge {
                    child: group("a"),
                    parent: group("b"),
                },
                GroupEdge {
                    child: group("b"),
                    parent: group("c"),
                },
            ],
            ExpansionLimits {
                max_depth: 1,
                max_nodes: 3,
                max_edges: 2,
            },
        );

        assert_eq!(result, Err(GroupExpansionError::DepthLimitExceeded));
    }

    #[test]
    fn clock_window_rejects_inverted_bounds() {
        assert_eq!(ClockWindow::new(20, 10), Err(ClockError::InvertedInterval));
    }
}
