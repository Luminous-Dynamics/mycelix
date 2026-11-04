#![forbid(unsafe_code)]
//! Independent, dependency-free reference model for multi-valued assurance
//! dependency inventories and transitive common-cause cut sets.
//!
//! This crate does not replace ASSURE-PORTFOLIO-CUT-001 or LEX-NET-030. It
//! specifies and tests the richer dependency projection required before the
//! existing v0.1 single-identity-per-dimension model can safely consume it.
//!
//! Digests are declared identifiers supplied by the caller. This crate does
//! not authenticate source bytes, real-world ownership, or inventory
//! completeness.

use std::collections::{BTreeMap, BTreeSet};

/// Upper bound on declared dependency nodes accepted by the reference model.
/// This prevents attacker-controlled graph size from exhausting memory or
/// relying on recursive call-stack depth.
pub const MAX_DEPENDENCY_NODES: usize = 100_000;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Digest(pub [u8; 32]);

impl Digest {
    pub const fn from_byte(byte: u8) -> Self {
        Self([byte; 32])
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum FaultDomainDimension {
    EvaluatorIdentity,
    EvaluatorProfileIdentity,
    ImplementationLineage,
    SpecificationLineage,
    ExpectedAnswerLineage,
    TestCorpusLineage,
    SourceEvidenceLineage,
    ToolchainDistribution,
    RepositoryControl,
    ExecutionProvider,
    RunnerImage,
    Runtime,
    InputAcquisition,
    OperatorControl,
    TrustRoot,
}

impl FaultDomainDimension {
    pub const ALL: [Self; 15] = [
        Self::EvaluatorIdentity,
        Self::EvaluatorProfileIdentity,
        Self::ImplementationLineage,
        Self::SpecificationLineage,
        Self::ExpectedAnswerLineage,
        Self::TestCorpusLineage,
        Self::SourceEvidenceLineage,
        Self::ToolchainDistribution,
        Self::RepositoryControl,
        Self::ExecutionProvider,
        Self::RunnerImage,
        Self::Runtime,
        Self::InputAcquisition,
        Self::OperatorControl,
        Self::TrustRoot,
    ];
}

/// Dimension-qualified identity. Equal digest bytes in different dimensions
/// are intentionally different fault-domain keys.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct FaultDomainKey {
    pub dimension: FaultDomainDimension,
    pub identity: Digest,
}

/// Whether compromising a dependency invalidates the role's evidence output.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum DependencyEffect {
    /// Under the current explicit model, compromise can invalidate the role.
    ClaimCritical,
    /// Retained and correlated, but not included in the current cut-set model.
    EvidentiaryLineage,
    /// Optional dependencies are not modeled as mandatory single points.
    Optional,
    /// Requires AND/OR or fallback fault semantics not implemented here.
    RedundantOrFallback,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceCurrentness {
    Current,
    Stale,
    Revoked,
    Unknown,
}

/// A concrete dependency node. Different node IDs may intentionally name the
/// same dimension-qualified fault domain; the audit canonicalizes by key.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DependencyNode {
    pub node_id: String,
    pub key: FaultDomainKey,
    pub effect: DependencyEffect,
    pub currentness: EvidenceCurrentness,
    pub evidence_refs: BTreeSet<Digest>,
    pub depends_on: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DimensionCoverage {
    /// The dependency inventory for this role/dimension is asserted complete.
    Complete,
    /// Some dependencies are known, but the inventory has an unknown remainder.
    Partial { rationale: String },
    /// The dimension or its dependencies cannot currently be established.
    Unknown { rationale: String },
    /// The dimension is outside this role's claim-specific threat model.
    NotApplicable { rationale: String },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RoleInventory {
    pub role: String,
    /// Root dependency node IDs; transitive dependencies are reached by edges.
    pub roots: BTreeSet<String>,
    /// Every one of the canonical 15 dimensions must be classified.
    pub coverage: BTreeMap<FaultDomainDimension, DimensionCoverage>,
}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct SeparationRequirement {
    pub left_role: String,
    pub right_role: String,
    pub dimension: FaultDomainDimension,
}

impl SeparationRequirement {
    pub fn new(
        left_role: impl Into<String>,
        right_role: impl Into<String>,
        dimension: FaultDomainDimension,
    ) -> Result<Self, AuditError> {
        let left = left_role.into();
        let right = right_role.into();
        if left.trim().is_empty() || right.trim().is_empty() {
            return Err(AuditError::EmptyRoleId);
        }
        if left == right {
            return Err(AuditError::SelfSeparation);
        }
        let (left_role, right_role) = if left < right {
            (left, right)
        } else {
            (right, left)
        };
        Ok(Self {
            left_role,
            right_role,
            dimension,
        })
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssurancePolicy {
    pub digest: Digest,
    pub required_roles: BTreeSet<String>,
    pub mandatory_separations: BTreeSet<SeparationRequirement>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DependencyGraph {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub profile_digest: Digest,
    pub policy_digest: Digest,
    pub manifest_digest: Digest,
    pub attack_model_digest: Digest,
    pub roles: Vec<RoleInventory>,
    pub nodes: Vec<DependencyNode>,
}

/// Exact caller-pinned input commitments and resource budget for one audit.
/// Keeping these bindings together prevents accidental positional swapping of
/// subject, profile, manifest and attack-model identities.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AuditBindings {
    pub pinned_policy_digest: Digest,
    pub expected_subject_digest: Digest,
    pub expected_profile_digest: Digest,
    pub expected_manifest_digest: Digest,
    pub expected_attack_model_digest: Digest,
    pub max_candidate_extensions: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuditDisposition {
    Complete,
    Incomplete,
    Unsupported,
    BudgetExceeded,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuditError {
    EmptyClaimId,
    EmptyRoleId,
    EmptyNodeId,
    EmptyRoots,
    DuplicateRoleId,
    DuplicateNodeId,
    DanglingRoot,
    DanglingDependency,
    DependencyCycle,
    DependencyGraphTooLarge,
    OrphanNode,
    MissingDimensionClassification,
    CompleteDimensionHasNoDependency,
    PartialCoverageHasNoKnownDependency,
    NotApplicableDimensionHasDependency,
    EmptyRationale,
    EmptyEvidenceReferenceSet,
    StaleEvidence,
    RevokedEvidence,
    EmptyRequiredRoles,
    RequiredRoleSetMismatch,
    SeparationRoleNotRequired,
    NonCanonicalSeparation,
    SelfSeparation,
    EmptyCriticalDependencySet,
    PolicyDigestMismatch,
    SubjectBindingMismatch,
    ProfileBindingMismatch,
    ManifestBindingMismatch,
    AttackModelBindingMismatch,
    ZeroCandidateBudget,
    CounterOverflow,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SharedDependency {
    pub key: FaultDomainKey,
    pub roles: Vec<String>,
    pub claim_critical_roles: Vec<String>,
    pub evidentiary_roles: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SeparationFinding {
    pub requirement: SeparationRequirement,
    pub disjoint_complete_inventories: bool,
    pub shared_keys: Vec<FaultDomainKey>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MinimalCutSetReport {
    pub disposition: AuditDisposition,
    pub minimal_cut_sets: Vec<BTreeSet<FaultDomainKey>>,
    pub minimum_cut_cardinality: Option<usize>,
    pub candidate_extensions_attempted: usize,
    pub max_candidate_extensions: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MultiDependencyAuditReport {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub profile_digest: Digest,
    pub policy_digest: Digest,
    pub manifest_digest: Digest,
    pub attack_model_digest: Digest,
    pub disposition: AuditDisposition,
    pub shared_dependencies: Vec<SharedDependency>,
    pub separation_findings: Vec<SeparationFinding>,
    pub incomplete_roles: Vec<String>,
    pub incomplete_dimensions: Vec<(String, FaultDomainDimension, String)>,
    pub unknown_evidence_nodes: Vec<(String, String)>,
    pub unsupported_roles: Vec<String>,
    pub unsupported_dependencies: Vec<(String, String, DependencyEffect)>,
    pub transitive_node_count_by_role: BTreeMap<String, usize>,
    pub cut_sets: MinimalCutSetReport,
}

#[derive(Clone, Debug)]
struct RoleProjection {
    all_keys: BTreeSet<FaultDomainKey>,
    critical_keys: BTreeSet<FaultDomainKey>,
    evidentiary_keys: BTreeSet<FaultDomainKey>,
    closure: BTreeSet<String>,
}

fn dependency_closure(
    roots: &BTreeSet<String>,
    nodes: &BTreeMap<String, &DependencyNode>,
) -> Result<BTreeSet<String>, AuditError> {
    let mut visited = BTreeSet::new();
    let mut pending: Vec<String> = roots.iter().rev().cloned().collect();
    while let Some(node_id) = pending.pop() {
        if !visited.insert(node_id.clone()) {
            continue;
        }
        let node = nodes.get(&node_id).ok_or(AuditError::DanglingRoot)?;
        for child in node.depends_on.iter().rev() {
            if !nodes.contains_key(child) {
                return Err(AuditError::DanglingDependency);
            }
            pending.push(child.clone());
        }
    }
    Ok(visited)
}

/// Iterative three-colour DFS. The explicit stack avoids making graph depth
/// an implicit process-stack limit for untrusted dependency manifests.
fn detect_cycles(nodes: &BTreeMap<String, &DependencyNode>) -> Result<(), AuditError> {
    let mut state = BTreeMap::<String, u8>::new();
    for start in nodes.keys() {
        if state.get(start).copied() == Some(2) {
            continue;
        }

        state.insert(start.clone(), 1);
        let mut stack = vec![(start.clone(), 0usize)];
        while !stack.is_empty() {
            let next_child = {
                let (node_id, next_index) = stack.last_mut().expect("non-empty DFS stack");
                let node = nodes.get(node_id).ok_or(AuditError::DanglingDependency)?;
                if *next_index < node.depends_on.len() {
                    let child = node.depends_on[*next_index].clone();
                    *next_index += 1;
                    Some(child)
                } else {
                    None
                }
            };

            let Some(child) = next_child else {
                let (finished, _) = stack.pop().expect("non-empty DFS stack");
                state.insert(finished, 2);
                continue;
            };

            if !nodes.contains_key(&child) {
                return Err(AuditError::DanglingDependency);
            }
            match state.get(&child).copied().unwrap_or(0) {
                1 => return Err(AuditError::DependencyCycle),
                2 => {}
                _ => {
                    state.insert(child.clone(), 1);
                    stack.push((child, 0));
                }
            }
        }
    }
    Ok(())
}

fn minimal_hitting_sets(
    role_sets: &[(String, BTreeSet<FaultDomainKey>)],
    max_candidate_extensions: usize,
) -> Result<MinimalCutSetReport, AuditError> {
    if max_candidate_extensions == 0 {
        return Err(AuditError::ZeroCandidateBudget);
    }
    let mut ordered_roles = role_sets.to_vec();
    ordered_roles.sort_by(|left, right| {
        left.1
            .len()
            .cmp(&right.1.len())
            .then_with(|| left.0.cmp(&right.0))
    });

    let mut candidates = vec![BTreeSet::<FaultDomainKey>::new()];
    let mut attempts = 0usize;
    for (_, role_keys) in ordered_roles {
        if role_keys.is_empty() {
            return Err(AuditError::EmptyCriticalDependencySet);
        }
        let mut expanded = BTreeSet::<BTreeSet<FaultDomainKey>>::new();
        for candidate in candidates {
            if candidate.iter().any(|key| role_keys.contains(key)) {
                expanded.insert(candidate);
                continue;
            }
            for key in &role_keys {
                attempts = attempts.checked_add(1).ok_or(AuditError::CounterOverflow)?;
                if attempts > max_candidate_extensions {
                    return Ok(MinimalCutSetReport {
                        disposition: AuditDisposition::BudgetExceeded,
                        minimal_cut_sets: Vec::new(),
                        minimum_cut_cardinality: None,
                        candidate_extensions_attempted: attempts,
                        max_candidate_extensions,
                    });
                }
                let mut next = candidate.clone();
                next.insert(*key);
                expanded.insert(next);
            }
        }
        let mut ordered: Vec<_> = expanded.into_iter().collect();
        ordered.sort_by(|left, right| left.len().cmp(&right.len()).then_with(|| left.cmp(right)));
        let mut minimal: Vec<BTreeSet<FaultDomainKey>> = Vec::new();
        for candidate in ordered {
            if !minimal
                .iter()
                .any(|existing| existing.is_subset(&candidate))
            {
                minimal.push(candidate);
            }
        }
        candidates = minimal;
    }

    candidates.sort_by(|left, right| left.len().cmp(&right.len()).then_with(|| left.cmp(right)));
    Ok(MinimalCutSetReport {
        disposition: AuditDisposition::Complete,
        minimum_cut_cardinality: candidates.first().map(BTreeSet::len),
        minimal_cut_sets: candidates,
        candidate_extensions_attempted: attempts,
        max_candidate_extensions,
    })
}

/// Validate the graph and calculate shared dependency identities and minimal
/// claim-critical cut sets. The caller supplies a policy digest pin and exact
/// expected subject/profile/manifest/attack-model commitments. The caller must
/// hash and authenticate the source bytes before constructing these values.
pub fn audit_multi_dependency_graph(
    graph: &DependencyGraph,
    policy: &AssurancePolicy,
    bindings: &AuditBindings,
) -> Result<MultiDependencyAuditReport, AuditError> {
    if graph.claim_id.trim().is_empty() {
        return Err(AuditError::EmptyClaimId);
    }
    if bindings.max_candidate_extensions == 0 {
        return Err(AuditError::ZeroCandidateBudget);
    }
    if graph.nodes.len() > MAX_DEPENDENCY_NODES {
        return Err(AuditError::DependencyGraphTooLarge);
    }
    if policy.required_roles.is_empty() {
        return Err(AuditError::EmptyRequiredRoles);
    }
    if policy.digest != bindings.pinned_policy_digest
        || graph.policy_digest != bindings.pinned_policy_digest
    {
        return Err(AuditError::PolicyDigestMismatch);
    }
    if graph.subject_digest != bindings.expected_subject_digest {
        return Err(AuditError::SubjectBindingMismatch);
    }
    if graph.profile_digest != bindings.expected_profile_digest {
        return Err(AuditError::ProfileBindingMismatch);
    }
    if graph.manifest_digest != bindings.expected_manifest_digest {
        return Err(AuditError::ManifestBindingMismatch);
    }
    if graph.attack_model_digest != bindings.expected_attack_model_digest {
        return Err(AuditError::AttackModelBindingMismatch);
    }

    let mut nodes = BTreeMap::<String, &DependencyNode>::new();
    for node in &graph.nodes {
        if node.node_id.trim().is_empty() {
            return Err(AuditError::EmptyNodeId);
        }
        if node.evidence_refs.is_empty() {
            return Err(AuditError::EmptyEvidenceReferenceSet);
        }
        match node.currentness {
            EvidenceCurrentness::Current => {}
            EvidenceCurrentness::Stale => return Err(AuditError::StaleEvidence),
            EvidenceCurrentness::Revoked => return Err(AuditError::RevokedEvidence),
            EvidenceCurrentness::Unknown => {}
        }
        if nodes.insert(node.node_id.clone(), node).is_some() {
            return Err(AuditError::DuplicateNodeId);
        }
    }
    detect_cycles(&nodes)?;

    let mut roles = BTreeMap::<String, &RoleInventory>::new();
    for role in &graph.roles {
        if role.role.trim().is_empty() {
            return Err(AuditError::EmptyRoleId);
        }
        if role.roots.is_empty() {
            return Err(AuditError::EmptyRoots);
        }
        if roles.insert(role.role.clone(), role).is_some() {
            return Err(AuditError::DuplicateRoleId);
        }
    }
    let expected_roles = &policy.required_roles;
    let actual_roles: BTreeSet<_> = roles.keys().cloned().collect();
    if &actual_roles != expected_roles {
        return Err(AuditError::RequiredRoleSetMismatch);
    }
    for separation in &policy.mandatory_separations {
        if separation.left_role >= separation.right_role {
            return Err(AuditError::NonCanonicalSeparation);
        }
        if !expected_roles.contains(&separation.left_role)
            || !expected_roles.contains(&separation.right_role)
        {
            return Err(AuditError::SeparationRoleNotRequired);
        }
    }

    let mut projections = BTreeMap::<String, RoleProjection>::new();
    let mut incomplete_roles = BTreeSet::new();
    let mut incomplete_dimensions = BTreeSet::new();
    let mut unknown_evidence_nodes = BTreeSet::new();
    let mut unsupported_roles = BTreeSet::new();
    let mut unsupported_dependencies = BTreeSet::new();
    let mut all_reachable = BTreeSet::new();

    for (role_name, role) in &roles {
        let closure = dependency_closure(&role.roots, &nodes)?;
        all_reachable.extend(closure.iter().cloned());

        let mut projection = RoleProjection {
            all_keys: BTreeSet::new(),
            critical_keys: BTreeSet::new(),
            evidentiary_keys: BTreeSet::new(),
            closure: closure.clone(),
        };

        for dimension in FaultDomainDimension::ALL {
            let Some(coverage) = role.coverage.get(&dimension) else {
                return Err(AuditError::MissingDimensionClassification);
            };
            let keys: Vec<_> = closure
                .iter()
                .filter_map(|node_id| nodes.get(node_id))
                .filter(|node| node.key.dimension == dimension)
                .map(|node| node.key)
                .collect();
            match coverage {
                DimensionCoverage::Complete => {
                    if keys.is_empty() {
                        return Err(AuditError::CompleteDimensionHasNoDependency);
                    }
                }
                DimensionCoverage::Partial { rationale } => {
                    if rationale.trim().is_empty() {
                        return Err(AuditError::EmptyRationale);
                    }
                    if keys.is_empty() {
                        return Err(AuditError::PartialCoverageHasNoKnownDependency);
                    }
                    incomplete_roles.insert(role_name.clone());
                    incomplete_dimensions.insert((role_name.clone(), dimension, rationale.clone()));
                }
                DimensionCoverage::Unknown { rationale } => {
                    if rationale.trim().is_empty() {
                        return Err(AuditError::EmptyRationale);
                    }
                    incomplete_roles.insert(role_name.clone());
                    incomplete_dimensions.insert((role_name.clone(), dimension, rationale.clone()));
                }
                DimensionCoverage::NotApplicable { rationale } => {
                    if rationale.trim().is_empty() {
                        return Err(AuditError::EmptyRationale);
                    }
                    if !keys.is_empty() {
                        return Err(AuditError::NotApplicableDimensionHasDependency);
                    }
                }
            }
        }

        for node_id in &closure {
            let node = nodes.get(node_id).ok_or(AuditError::DanglingDependency)?;
            projection.all_keys.insert(node.key);
            match node.currentness {
                EvidenceCurrentness::Current => {}
                EvidenceCurrentness::Stale => return Err(AuditError::StaleEvidence),
                EvidenceCurrentness::Revoked => return Err(AuditError::RevokedEvidence),
                EvidenceCurrentness::Unknown => {
                    incomplete_roles.insert(role_name.clone());
                    unknown_evidence_nodes.insert((role_name.clone(), node_id.clone()));
                }
            }
            match node.effect {
                DependencyEffect::ClaimCritical => {
                    projection.critical_keys.insert(node.key);
                }
                DependencyEffect::EvidentiaryLineage => {
                    projection.evidentiary_keys.insert(node.key);
                }
                DependencyEffect::Optional | DependencyEffect::RedundantOrFallback => {
                    unsupported_roles.insert(role_name.clone());
                    unsupported_dependencies.insert((
                        role_name.clone(),
                        node_id.clone(),
                        node.effect,
                    ));
                }
            }
        }
        if projection.critical_keys.is_empty() {
            incomplete_roles.insert(role_name.clone());
        }
        projections.insert(role_name.clone(), projection);
    }

    let orphan_nodes: BTreeSet<_> = nodes
        .keys()
        .filter(|node_id| !all_reachable.contains(*node_id))
        .cloned()
        .collect();
    if !orphan_nodes.is_empty() {
        return Err(AuditError::OrphanNode);
    }

    let mut identity_roles = BTreeMap::<FaultDomainKey, BTreeSet<String>>::new();
    let mut critical_roles = BTreeMap::<FaultDomainKey, BTreeSet<String>>::new();
    let mut evidentiary_roles = BTreeMap::<FaultDomainKey, BTreeSet<String>>::new();
    for (role, projection) in &projections {
        for key in &projection.all_keys {
            identity_roles.entry(*key).or_default().insert(role.clone());
        }
        for key in &projection.critical_keys {
            critical_roles.entry(*key).or_default().insert(role.clone());
        }
        for key in &projection.evidentiary_keys {
            evidentiary_roles
                .entry(*key)
                .or_default()
                .insert(role.clone());
        }
    }

    let shared_dependencies: Vec<_> = identity_roles
        .into_iter()
        .filter_map(|(key, role_set)| {
            (role_set.len() > 1).then(|| SharedDependency {
                key,
                roles: role_set.iter().cloned().collect(),
                claim_critical_roles: critical_roles
                    .get(&key)
                    .map(|set| set.iter().cloned().collect())
                    .unwrap_or_default(),
                evidentiary_roles: evidentiary_roles
                    .get(&key)
                    .map(|set| set.iter().cloned().collect())
                    .unwrap_or_default(),
            })
        })
        .collect();

    let mut separation_findings = Vec::new();
    for requirement in &policy.mandatory_separations {
        let left_role = roles
            .get(&requirement.left_role)
            .ok_or(AuditError::SeparationRoleNotRequired)?;
        let right_role = roles
            .get(&requirement.right_role)
            .ok_or(AuditError::SeparationRoleNotRequired)?;
        let left_coverage = left_role
            .coverage
            .get(&requirement.dimension)
            .ok_or(AuditError::MissingDimensionClassification)?;
        let right_coverage = right_role
            .coverage
            .get(&requirement.dimension)
            .ok_or(AuditError::MissingDimensionClassification)?;
        let left: BTreeSet<_> = projections[&requirement.left_role]
            .all_keys
            .iter()
            .filter(|key| key.dimension == requirement.dimension)
            .copied()
            .collect();
        let right: BTreeSet<_> = projections[&requirement.right_role]
            .all_keys
            .iter()
            .filter(|key| key.dimension == requirement.dimension)
            .copied()
            .collect();
        let shared: Vec<_> = left.intersection(&right).copied().collect();
        let inventories_complete = matches!(left_coverage, DimensionCoverage::Complete)
            && matches!(right_coverage, DimensionCoverage::Complete);
        let disjoint_complete_inventories = inventories_complete && shared.is_empty();
        if !disjoint_complete_inventories {
            incomplete_roles.insert(requirement.left_role.clone());
            incomplete_roles.insert(requirement.right_role.clone());
        }
        separation_findings.push(SeparationFinding {
            requirement: requirement.clone(),
            disjoint_complete_inventories,
            shared_keys: shared,
        });
    }

    let disposition = if !unsupported_roles.is_empty() {
        AuditDisposition::Unsupported
    } else if !incomplete_roles.is_empty() {
        AuditDisposition::Incomplete
    } else {
        AuditDisposition::Complete
    };

    let empty_cut_sets = MinimalCutSetReport {
        disposition,
        minimal_cut_sets: Vec::new(),
        minimum_cut_cardinality: None,
        candidate_extensions_attempted: 0,
        max_candidate_extensions: bindings.max_candidate_extensions,
    };
    let cut_sets = if disposition == AuditDisposition::Complete {
        let role_sets: Vec<_> = projections
            .iter()
            .map(|(role, projection)| (role.clone(), projection.critical_keys.clone()))
            .collect();
        minimal_hitting_sets(&role_sets, bindings.max_candidate_extensions)?
    } else {
        empty_cut_sets
    };

    Ok(MultiDependencyAuditReport {
        claim_id: graph.claim_id.clone(),
        subject_digest: graph.subject_digest,
        profile_digest: graph.profile_digest,
        policy_digest: graph.policy_digest,
        manifest_digest: graph.manifest_digest,
        attack_model_digest: graph.attack_model_digest,
        disposition: cut_sets.disposition,
        shared_dependencies,
        separation_findings,
        incomplete_roles: incomplete_roles.into_iter().collect(),
        incomplete_dimensions: incomplete_dimensions.into_iter().collect(),
        unknown_evidence_nodes: unknown_evidence_nodes.into_iter().collect(),
        unsupported_roles: unsupported_roles.into_iter().collect(),
        unsupported_dependencies: unsupported_dependencies.into_iter().collect(),
        transitive_node_count_by_role: projections
            .into_iter()
            .map(|(role, projection)| (role, projection.closure.len()))
            .collect(),
        cut_sets,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(dimension: FaultDomainDimension, identity: u8) -> FaultDomainKey {
        FaultDomainKey {
            dimension,
            identity: Digest::from_byte(identity),
        }
    }

    fn node(
        id: &str,
        dimension: FaultDomainDimension,
        identity: u8,
        effect: DependencyEffect,
        depends_on: &[&str],
    ) -> DependencyNode {
        DependencyNode {
            node_id: id.to_owned(),
            key: key(dimension, identity),
            effect,
            currentness: EvidenceCurrentness::Current,
            evidence_refs: BTreeSet::from([Digest::from_byte(identity)]),
            depends_on: depends_on.iter().map(|item| (*item).to_owned()).collect(),
        }
    }

    fn coverage_for(
        dimensions: &[FaultDomainDimension],
    ) -> BTreeMap<FaultDomainDimension, DimensionCoverage> {
        FaultDomainDimension::ALL
            .into_iter()
            .map(|dimension| {
                let coverage = if dimensions.contains(&dimension) {
                    DimensionCoverage::Complete
                } else {
                    DimensionCoverage::NotApplicable {
                        rationale: "outside this claim's declared attack surface".to_owned(),
                    }
                };
                (dimension, coverage)
            })
            .collect()
    }

    fn role(role: &str, roots: &[&str], dimensions: &[FaultDomainDimension]) -> RoleInventory {
        RoleInventory {
            role: role.to_owned(),
            roots: roots.iter().map(|item| (*item).to_owned()).collect(),
            coverage: coverage_for(dimensions),
        }
    }

    fn graph(roles: Vec<RoleInventory>, nodes: Vec<DependencyNode>) -> DependencyGraph {
        DependencyGraph {
            claim_id: "claim-multi".to_owned(),
            subject_digest: Digest::from_byte(1),
            profile_digest: Digest::from_byte(2),
            policy_digest: Digest::from_byte(3),
            manifest_digest: Digest::from_byte(4),
            attack_model_digest: Digest::from_byte(5),
            roles,
            nodes,
        }
    }

    fn policy(roles: &[&str], separations: Vec<SeparationRequirement>) -> AssurancePolicy {
        AssurancePolicy {
            digest: Digest::from_byte(3),
            required_roles: roles.iter().map(|role| (*role).to_owned()).collect(),
            mandatory_separations: separations.into_iter().collect(),
        }
    }

    fn expected_bindings() -> AuditBindings {
        AuditBindings {
            pinned_policy_digest: Digest::from_byte(3),
            expected_subject_digest: Digest::from_byte(1),
            expected_profile_digest: Digest::from_byte(2),
            expected_manifest_digest: Digest::from_byte(4),
            expected_attack_model_digest: Digest::from_byte(5),
            max_candidate_extensions: 100_000,
        }
    }

    fn audit(
        graph: &DependencyGraph,
        policy: &AssurancePolicy,
        budget: usize,
    ) -> MultiDependencyAuditReport {
        let mut bindings = expected_bindings();
        bindings.max_candidate_extensions = budget;
        audit_multi_dependency_graph(graph, policy, &bindings).expect("valid graph structure")
    }

    #[test]
    fn multiple_identities_in_one_dimension_are_preserved() {
        let nodes = vec![
            node(
                "a-host",
                FaultDomainDimension::ExecutionProvider,
                10,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "a-runner",
                FaultDomainDimension::ExecutionProvider,
                11,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b-host",
                FaultDomainDimension::ExecutionProvider,
                20,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b-runner",
                FaultDomainDimension::ExecutionProvider,
                21,
                DependencyEffect::ClaimCritical,
                &[],
            ),
        ];
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["a-host", "a-runner"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
                role(
                    "independent",
                    &["b-host", "b-runner"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            nodes,
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        assert_eq!(report.disposition, AuditDisposition::Complete);
        assert_eq!(report.cut_sets.minimum_cut_cardinality, Some(2));
        assert_eq!(report.cut_sets.minimal_cut_sets.len(), 4);
        assert!(report.shared_dependencies.is_empty());
    }

    #[test]
    fn partial_overlap_in_multi_value_dimension_is_not_hidden() {
        let shared = key(FaultDomainDimension::ExecutionProvider, 99);
        let nodes = vec![
            node(
                "a-private",
                FaultDomainDimension::ExecutionProvider,
                1,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "a-shared",
                FaultDomainDimension::ExecutionProvider,
                99,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b-private",
                FaultDomainDimension::ExecutionProvider,
                2,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b-shared-alias",
                FaultDomainDimension::ExecutionProvider,
                99,
                DependencyEffect::ClaimCritical,
                &[],
            ),
        ];
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["a-private", "a-shared"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
                role(
                    "independent",
                    &["b-private", "b-shared-alias"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            nodes,
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        assert!(
            report
                .shared_dependencies
                .iter()
                .any(|dependency| dependency.key == shared)
        );
        assert_eq!(report.cut_sets.minimum_cut_cardinality, Some(1));
        assert!(
            report
                .cut_sets
                .minimal_cut_sets
                .contains(&BTreeSet::from([shared]))
        );
    }

    #[test]
    fn transitive_shared_root_is_detected_through_dag_closure() {
        let nodes = vec![
            node(
                "formal-service",
                FaultDomainDimension::Runtime,
                1,
                DependencyEffect::ClaimCritical,
                &["shared-host"],
            ),
            node(
                "independent-service",
                FaultDomainDimension::Runtime,
                2,
                DependencyEffect::ClaimCritical,
                &["shared-host"],
            ),
            node(
                "shared-host",
                FaultDomainDimension::ExecutionProvider,
                99,
                DependencyEffect::ClaimCritical,
                &[],
            ),
        ];
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["formal-service"],
                    &[
                        FaultDomainDimension::Runtime,
                        FaultDomainDimension::ExecutionProvider,
                    ],
                ),
                role(
                    "independent",
                    &["independent-service"],
                    &[
                        FaultDomainDimension::Runtime,
                        FaultDomainDimension::ExecutionProvider,
                    ],
                ),
            ],
            nodes,
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        assert_eq!(report.transitive_node_count_by_role["formal"], 2);
        assert_eq!(report.transitive_node_count_by_role["independent"], 2);
        assert!(
            report
                .shared_dependencies
                .iter()
                .any(|item| item.key == key(FaultDomainDimension::ExecutionProvider, 99))
        );
    }

    #[test]
    fn known_shared_dependencies_do_not_hide_unknown_remainder() {
        let mut graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                node(
                    "a",
                    FaultDomainDimension::ExecutionProvider,
                    99,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    99,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
            ],
        );
        graph.roles[0].coverage.insert(
            FaultDomainDimension::ExecutionProvider,
            DimensionCoverage::Partial {
                rationale: "unresolved host-side dependencies".to_owned(),
            },
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        assert_eq!(report.disposition, AuditDisposition::Incomplete);
        assert!(report.cut_sets.minimal_cut_sets.is_empty());
        assert_eq!(report.cut_sets.minimum_cut_cardinality, None);
        assert!(
            report
                .shared_dependencies
                .iter()
                .any(|item| item.key == key(FaultDomainDimension::ExecutionProvider, 99))
        );
    }

    #[test]
    fn unknown_dependency_evidence_blocks_numeric_cut_sets() {
        let mut graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                node(
                    "a",
                    FaultDomainDimension::ExecutionProvider,
                    1,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    2,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
            ],
        );
        let mut unknown = graph.nodes[0].clone();
        unknown.node_id = "unknown-root".to_owned();
        unknown.key.identity = Digest::from_byte(77);
        unknown.currentness = EvidenceCurrentness::Unknown;
        graph.nodes.push(unknown);
        graph.roles[0].roots.insert("unknown-root".to_owned());
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        assert_eq!(report.disposition, AuditDisposition::Incomplete);
        assert!(report.cut_sets.minimal_cut_sets.is_empty());
        assert_eq!(report.cut_sets.minimum_cut_cardinality, None);
    }

    #[test]
    fn optional_or_fallback_paths_are_unsupported_under_any_one_compromise_model() {
        let nodes = vec![
            node(
                "a",
                FaultDomainDimension::ExecutionProvider,
                1,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "a-fallback",
                FaultDomainDimension::ExecutionProvider,
                2,
                DependencyEffect::RedundantOrFallback,
                &[],
            ),
            node(
                "b",
                FaultDomainDimension::ExecutionProvider,
                3,
                DependencyEffect::ClaimCritical,
                &[],
            ),
        ];
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["a", "a-fallback"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            nodes,
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        assert_eq!(report.disposition, AuditDisposition::Unsupported);
        assert!(report.cut_sets.minimal_cut_sets.is_empty());
        assert_eq!(report.cut_sets.minimum_cut_cardinality, None);
    }

    #[test]
    fn evidentiary_lineage_is_visible_but_not_misclassified_as_claim_critical() {
        let shared_evidence = key(FaultDomainDimension::ExpectedAnswerLineage, 90);
        let nodes = vec![
            node(
                "a-critical",
                FaultDomainDimension::ImplementationLineage,
                1,
                DependencyEffect::ClaimCritical,
                &["shared-evidence"],
            ),
            node(
                "b-critical",
                FaultDomainDimension::ImplementationLineage,
                2,
                DependencyEffect::ClaimCritical,
                &["shared-evidence"],
            ),
            node(
                "shared-evidence",
                FaultDomainDimension::ExpectedAnswerLineage,
                90,
                DependencyEffect::EvidentiaryLineage,
                &[],
            ),
        ];
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["a-critical"],
                    &[
                        FaultDomainDimension::ImplementationLineage,
                        FaultDomainDimension::ExpectedAnswerLineage,
                    ],
                ),
                role(
                    "independent",
                    &["b-critical"],
                    &[
                        FaultDomainDimension::ImplementationLineage,
                        FaultDomainDimension::ExpectedAnswerLineage,
                    ],
                ),
            ],
            nodes,
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        let shared = report
            .shared_dependencies
            .iter()
            .find(|item| item.key == shared_evidence)
            .expect("shared evidence");
        assert_eq!(
            shared.evidentiary_roles,
            vec!["formal".to_owned(), "independent".to_owned()]
        );
        assert!(shared.claim_critical_roles.is_empty());
        assert_eq!(report.cut_sets.minimum_cut_cardinality, Some(2));
    }

    #[test]
    fn same_digest_in_different_dimensions_is_not_a_shared_domain() {
        let same = Digest::from_byte(42);
        let nodes = vec![
            DependencyNode {
                node_id: "a".into(),
                key: FaultDomainKey {
                    dimension: FaultDomainDimension::Runtime,
                    identity: same,
                },
                effect: DependencyEffect::ClaimCritical,
                currentness: EvidenceCurrentness::Current,
                evidence_refs: BTreeSet::from([Digest::from_byte(1)]),
                depends_on: vec![],
            },
            DependencyNode {
                node_id: "b".into(),
                key: FaultDomainKey {
                    dimension: FaultDomainDimension::ExecutionProvider,
                    identity: same,
                },
                effect: DependencyEffect::ClaimCritical,
                currentness: EvidenceCurrentness::Current,
                evidence_refs: BTreeSet::from([Digest::from_byte(2)]),
                depends_on: vec![],
            },
        ];
        let graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::Runtime]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            nodes,
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        assert!(report.shared_dependencies.is_empty());
    }

    #[test]
    fn required_pairwise_separation_uses_full_sets_not_one_digest() {
        let nodes = vec![
            node(
                "a1",
                FaultDomainDimension::ImplementationLineage,
                1,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "a2",
                FaultDomainDimension::ImplementationLineage,
                9,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b1",
                FaultDomainDimension::ImplementationLineage,
                2,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b2",
                FaultDomainDimension::ImplementationLineage,
                9,
                DependencyEffect::ClaimCritical,
                &[],
            ),
        ];
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["a1", "a2"],
                    &[FaultDomainDimension::ImplementationLineage],
                ),
                role(
                    "independent",
                    &["b1", "b2"],
                    &[FaultDomainDimension::ImplementationLineage],
                ),
            ],
            nodes,
        );
        let separation = SeparationRequirement::new(
            "formal",
            "independent",
            FaultDomainDimension::ImplementationLineage,
        )
        .expect("valid");
        let policy = policy(&["formal", "independent"], vec![separation]);
        let report = audit(&graph, &policy, 1000);
        assert_eq!(report.disposition, AuditDisposition::Incomplete);
        assert_eq!(report.separation_findings.len(), 1);
        assert_eq!(
            report.separation_findings[0].shared_keys,
            vec![key(FaultDomainDimension::ImplementationLineage, 9)]
        );
        assert!(report.cut_sets.minimal_cut_sets.is_empty());
    }

    #[test]
    fn cycle_and_dangling_edges_fail_closed() {
        let cyclic = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                node(
                    "a",
                    FaultDomainDimension::ExecutionProvider,
                    1,
                    DependencyEffect::ClaimCritical,
                    &["b"],
                ),
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    2,
                    DependencyEffect::ClaimCritical,
                    &["a"],
                ),
            ],
        );
        assert_eq!(
            audit_multi_dependency_graph(
                &cyclic,
                &policy(&["formal", "independent"], vec![]),
                &expected_bindings()
            ),
            Err(AuditError::DependencyCycle)
        );

        let dangling = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                node(
                    "a",
                    FaultDomainDimension::ExecutionProvider,
                    1,
                    DependencyEffect::ClaimCritical,
                    &["missing"],
                ),
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    2,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
            ],
        );
        assert_eq!(
            audit_multi_dependency_graph(
                &dangling,
                &policy(&["formal", "independent"], vec![]),
                &expected_bindings()
            ),
            Err(AuditError::DanglingDependency)
        );
    }

    fn brute_force_reference(
        role_sets: &[(String, BTreeSet<FaultDomainKey>)],
    ) -> Vec<BTreeSet<FaultDomainKey>> {
        let universe: Vec<_> = role_sets
            .iter()
            .flat_map(|(_, keys)| keys.iter().copied())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        assert!(universe.len() < usize::BITS as usize);
        let mut hitting = Vec::new();
        for mask in 1usize..(1usize << universe.len()) {
            let candidate: BTreeSet<_> = universe
                .iter()
                .enumerate()
                .filter_map(|(index, key)| ((mask & (1usize << index)) != 0).then_some(*key))
                .collect();
            if role_sets
                .iter()
                .all(|(_, keys)| candidate.iter().any(|key| keys.contains(key)))
            {
                hitting.push(candidate);
            }
        }
        hitting.sort_by(|a, b| a.len().cmp(&b.len()).then_with(|| a.cmp(b)));
        let mut minimal = Vec::new();
        for candidate in hitting {
            if !minimal
                .iter()
                .any(|existing: &BTreeSet<FaultDomainKey>| existing.is_subset(&candidate))
            {
                minimal.push(candidate);
            }
        }
        minimal
    }

    #[test]
    fn incremental_search_matches_independent_brute_force_on_128_graphs() {
        let dims = [
            FaultDomainDimension::ExecutionProvider,
            FaultDomainDimension::ImplementationLineage,
            FaultDomainDimension::ExpectedAnswerLineage,
        ];
        for case in 0..128usize {
            let role_count = 2 + case % 4;
            let mut nodes = Vec::new();
            let mut roles = Vec::new();
            let mut role_sets = Vec::new();
            for role_index in 0..role_count {
                let role_name = format!("role-{role_index}");
                let mut roots = Vec::new();
                let mut keys = BTreeSet::new();
                for (dim_index, dimension) in dims.iter().copied().enumerate() {
                    if (case + role_index * 3 + dim_index) % 3 != 0 {
                        let identity = if (case + role_index + dim_index) % 2 == 0 {
                            (case + dim_index + 1) as u8
                        } else {
                            (80 + role_index * 8 + dim_index) as u8
                        };
                        let id = format!("n-{role_index}-{dim_index}");
                        roots.push(id.clone());
                        nodes.push(node(
                            &id,
                            dimension,
                            identity,
                            DependencyEffect::ClaimCritical,
                            &[],
                        ));
                        keys.insert(key(dimension, identity));
                    }
                }
                if keys.is_empty() {
                    let dimension = dims[role_index % dims.len()];
                    let id = format!("n-{role_index}-fallback");
                    roots.push(id.clone());
                    nodes.push(node(
                        &id,
                        dimension,
                        200 + role_index as u8,
                        DependencyEffect::ClaimCritical,
                        &[],
                    ));
                    keys.insert(key(dimension, 200 + role_index as u8));
                }
                let dimensions: Vec<_> = dims
                    .iter()
                    .copied()
                    .filter(|d| keys.iter().any(|k| k.dimension == *d))
                    .collect();
                let root_refs: Vec<_> = roots.iter().map(String::as_str).collect();
                roles.push(role(&role_name, &root_refs, &dimensions));
                role_sets.push((role_name, keys));
            }
            let role_names: Vec<String> = (0..role_count)
                .map(|index| format!("role-{index}"))
                .collect();
            let role_refs: Vec<_> = role_names.iter().map(String::as_str).collect();
            let graph = graph(roles, nodes);
            let policy = policy(&role_refs, vec![]);
            let expected = brute_force_reference(&role_sets);
            let actual = audit(&graph, &policy, 100_000);
            assert_eq!(
                actual.disposition,
                AuditDisposition::Complete,
                "case {case}"
            );
            assert_eq!(actual.cut_sets.minimal_cut_sets, expected, "case {case}");
        }
    }

    #[test]
    fn budget_exhaustion_never_returns_partial_cut_sets() {
        let nodes = vec![
            node(
                "a1",
                FaultDomainDimension::ExecutionProvider,
                1,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "a2",
                FaultDomainDimension::ExecutionProvider,
                2,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b1",
                FaultDomainDimension::ExecutionProvider,
                3,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b2",
                FaultDomainDimension::ExecutionProvider,
                4,
                DependencyEffect::ClaimCritical,
                &[],
            ),
        ];
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["a1", "a2"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
                role(
                    "independent",
                    &["b1", "b2"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            nodes,
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1);
        assert_eq!(report.disposition, AuditDisposition::BudgetExceeded);
        assert!(report.cut_sets.minimal_cut_sets.is_empty());
        assert_eq!(report.cut_sets.minimum_cut_cardinality, None);
    }

    #[test]
    fn stale_and_revoked_source_evidence_are_rejected() {
        let mut stale = node(
            "a",
            FaultDomainDimension::ExecutionProvider,
            1,
            DependencyEffect::ClaimCritical,
            &[],
        );
        stale.currentness = EvidenceCurrentness::Stale;
        let graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                stale,
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    2,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
            ],
        );
        assert_eq!(
            audit_multi_dependency_graph(
                &graph,
                &policy(&["formal", "independent"], vec![]),
                &expected_bindings()
            ),
            Err(AuditError::StaleEvidence)
        );
    }

    #[test]
    fn duplicate_node_ids_and_orphan_nodes_are_rejected() {
        let duplicate = node(
            "same",
            FaultDomainDimension::ExecutionProvider,
            1,
            DependencyEffect::ClaimCritical,
            &[],
        );
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["same"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
                role(
                    "independent",
                    &["same"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![duplicate.clone(), duplicate],
        );
        assert_eq!(
            audit_multi_dependency_graph(
                &graph,
                &policy(&["formal", "independent"], vec![]),
                &expected_bindings()
            ),
            Err(AuditError::DuplicateNodeId)
        );
    }

    #[test]
    fn wrong_subject_or_policy_binding_is_rejected() {
        let nodes = vec![
            node(
                "a",
                FaultDomainDimension::ExecutionProvider,
                1,
                DependencyEffect::ClaimCritical,
                &[],
            ),
            node(
                "b",
                FaultDomainDimension::ExecutionProvider,
                2,
                DependencyEffect::ClaimCritical,
                &[],
            ),
        ];
        let graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            nodes,
        );
        assert_eq!(
            audit_multi_dependency_graph(
                &graph,
                &policy(&["formal", "independent"], vec![]),
                &AuditBindings {
                    expected_subject_digest: Digest::from_byte(8),
                    ..expected_bindings()
                }
            ),
            Err(AuditError::SubjectBindingMismatch)
        );
        assert_eq!(
            audit_multi_dependency_graph(
                &graph,
                &policy(&["formal", "independent"], vec![]),
                &AuditBindings {
                    pinned_policy_digest: Digest::from_byte(4),
                    ..expected_bindings()
                }
            ),
            Err(AuditError::PolicyDigestMismatch)
        );
    }

    #[test]
    fn unknown_dimension_coverage_keeps_known_nodes_but_blocks_cut_sets() {
        let mut graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                node(
                    "a",
                    FaultDomainDimension::ExecutionProvider,
                    1,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    2,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
            ],
        );
        graph.roles[0].coverage.insert(
            FaultDomainDimension::ExecutionProvider,
            DimensionCoverage::Unknown {
                rationale: "host inventory not acquired".to_owned(),
            },
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 1000);
        assert_eq!(report.disposition, AuditDisposition::Incomplete);
        assert!(report.cut_sets.minimal_cut_sets.is_empty());
        assert_eq!(report.cut_sets.minimum_cut_cardinality, None);
    }

    #[test]
    fn missing_dimension_classification_is_rejected() {
        let mut graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                node(
                    "a",
                    FaultDomainDimension::ExecutionProvider,
                    1,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    2,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
            ],
        );
        graph.roles[0]
            .coverage
            .remove(&FaultDomainDimension::TrustRoot);
        assert_eq!(
            audit_multi_dependency_graph(
                &graph,
                &policy(&["formal", "independent"], vec![]),
                &expected_bindings()
            ),
            Err(AuditError::MissingDimensionClassification)
        );
    }

    #[test]
    fn duplicate_and_reordered_dependency_edges_do_not_change_closure_or_cut_sets() {
        let base_nodes = vec![
            node(
                "a-private",
                FaultDomainDimension::ImplementationLineage,
                1,
                DependencyEffect::ClaimCritical,
                &["shared"],
            ),
            node(
                "b-private",
                FaultDomainDimension::ImplementationLineage,
                2,
                DependencyEffect::ClaimCritical,
                &["shared"],
            ),
            node(
                "shared",
                FaultDomainDimension::ExecutionProvider,
                99,
                DependencyEffect::ClaimCritical,
                &[],
            ),
        ];
        let base = graph(
            vec![
                role(
                    "formal",
                    &["a-private"],
                    &[
                        FaultDomainDimension::ImplementationLineage,
                        FaultDomainDimension::ExecutionProvider,
                    ],
                ),
                role(
                    "independent",
                    &["b-private"],
                    &[
                        FaultDomainDimension::ImplementationLineage,
                        FaultDomainDimension::ExecutionProvider,
                    ],
                ),
            ],
            base_nodes.clone(),
        );
        let mut duplicated = base.clone();
        duplicated.nodes[0].depends_on.push("shared".to_string());
        duplicated.nodes[0].depends_on.reverse();
        duplicated.nodes[1].depends_on.reverse();

        let clean_report = audit(&base, &policy(&["formal", "independent"], vec![]), 1000);
        let mutated_report = audit(
            &duplicated,
            &policy(&["formal", "independent"], vec![]),
            1000,
        );
        assert_eq!(
            clean_report.shared_dependencies,
            mutated_report.shared_dependencies
        );
        assert_eq!(
            clean_report.cut_sets.minimal_cut_sets,
            mutated_report.cut_sets.minimal_cut_sets
        );
        assert_eq!(
            clean_report.transitive_node_count_by_role,
            mutated_report.transitive_node_count_by_role
        );
    }

    #[test]
    fn orphan_dependency_node_is_rejected() {
        let graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                node(
                    "a",
                    FaultDomainDimension::ExecutionProvider,
                    1,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    2,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
                node(
                    "orphan",
                    FaultDomainDimension::TrustRoot,
                    3,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
            ],
        );
        assert_eq!(
            audit_multi_dependency_graph(
                &graph,
                &policy(&["formal", "independent"], vec![]),
                &expected_bindings()
            ),
            Err(AuditError::OrphanNode)
        );
    }

    #[test]
    fn revoked_evidence_is_rejected() {
        let mut revoked = node(
            "a",
            FaultDomainDimension::ExecutionProvider,
            1,
            DependencyEffect::ClaimCritical,
            &[],
        );
        revoked.currentness = EvidenceCurrentness::Revoked;
        let graph = graph(
            vec![
                role("formal", &["a"], &[FaultDomainDimension::ExecutionProvider]),
                role(
                    "independent",
                    &["b"],
                    &[FaultDomainDimension::ExecutionProvider],
                ),
            ],
            vec![
                revoked,
                node(
                    "b",
                    FaultDomainDimension::ExecutionProvider,
                    2,
                    DependencyEffect::ClaimCritical,
                    &[],
                ),
            ],
        );
        assert_eq!(
            audit_multi_dependency_graph(
                &graph,
                &policy(&["formal", "independent"], vec![]),
                &expected_bindings()
            ),
            Err(AuditError::RevokedEvidence)
        );
    }

    #[test]
    fn long_dependency_chain_uses_iterative_cycle_detection() {
        let count = 20_000usize;
        let mut nodes = Vec::with_capacity(count);
        for index in 0..count {
            let id = format!("node-{index}");
            let next = if index + 1 < count {
                vec![format!("node-{}", index + 1)]
            } else {
                Vec::new()
            };
            nodes.push(DependencyNode {
                node_id: id,
                key: key(
                    if index % 2 == 0 {
                        FaultDomainDimension::ExecutionProvider
                    } else {
                        FaultDomainDimension::Runtime
                    },
                    (index % 251) as u8,
                ),
                effect: DependencyEffect::ClaimCritical,
                currentness: EvidenceCurrentness::Current,
                evidence_refs: BTreeSet::from([Digest::from_byte((index % 251) as u8)]),
                depends_on: next,
            });
        }
        let graph = graph(
            vec![
                role(
                    "formal",
                    &["node-0"],
                    &[
                        FaultDomainDimension::ExecutionProvider,
                        FaultDomainDimension::Runtime,
                    ],
                ),
                role(
                    "independent",
                    &["node-19999"],
                    &[FaultDomainDimension::Runtime],
                ),
            ],
            nodes,
        );
        let report = audit(&graph, &policy(&["formal", "independent"], vec![]), 10_000);
        assert_eq!(report.transitive_node_count_by_role["formal"], count);
        assert_eq!(report.disposition, AuditDisposition::Complete);
    }
}
