//! D6X qualified semantic dependency closure reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! D6X is narrower than graph reachability. A closure is the deterministic
//! result of applying a named dependency policy to a qualified projection.
//! Provenance/custody and unrelated graph material are not dependencies unless
//! the profile explicitly makes them dependencies.
use crate::canonical_derivation_receipt::{
    canonical_sha256, DerivationProfileV1, QualifiedProjectionV1, SemanticEnvironmentV1,
    D6S_CLAIM_CEILING,
};
use crate::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

pub const D6X_SCHEMA_VERSION: &str = "D6X-1";
pub const D6X_ALGORITHM_VERSION: &str = "D6X-CLOSURE-4";
pub const D6X_CANONICALIZATION_VERSION: &str = "D6S-CANON-1";
pub const D6X_CLAIM_CEILING: &str = D6S_CLAIM_CEILING;

fn non_empty(v: &str) -> bool { !v.trim().is_empty() }

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum DependencyClosureStatusV1 {
    Complete,
    BlockedMissingDependency,
    BlockedCurrentness,
    BlockedResourceLimit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum DependencyCurrentnessV1 { Any, CurrentOnly }

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct DependencyRuleV1 {
    pub edge_kind: ClaimGraphEdgeKindV1,
    pub from_kind: Option<ClaimGraphNodeKindV1>,
    pub to_kind: Option<ClaimGraphNodeKindV1>,
    pub currentness: DependencyCurrentnessV1,
}
impl DependencyRuleV1 {
    pub fn structurally_valid(&self) -> bool {
        self.from_kind.is_some() || self.to_kind.is_some() || self.edge_kind.is_support_semantic()
    }
    fn matches(&self, from: ClaimGraphNodeKindV1, to: ClaimGraphNodeKindV1, edge: ClaimGraphEdgeKindV1) -> bool {
        self.edge_kind == edge
            && self.from_kind.map_or(true, |k| k == from)
            && self.to_kind.map_or(true, |k| k == to)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DependencyClosureProfileV1 {
    pub profile_id: String,
    pub version: String,
    pub root_node_ids: BTreeSet<String>,
    pub required_node_ids: BTreeSet<String>,
    /// Exact D6P receipt commitments required by this semantic closure profile.
    pub required_d6p_receipt_commitments: BTreeSet<String>,
    pub rules: BTreeSet<DependencyRuleV1>,
    pub excluded_boundary_policy: String,
    pub max_nodes: u32,
    pub max_edges: u32,
    pub claim_ceiling: String,
}
impl DependencyClosureProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.profile_id)
            && non_empty(&self.version)
            && !self.root_node_ids.is_empty()
            && !self.rules.is_empty()
            && self.rules.iter().all(DependencyRuleV1::structurally_valid)
            && self.root_node_ids.iter().all(|v| non_empty(v))
            && self.required_node_ids.iter().all(|v| non_empty(v))
            && self.required_d6p_receipt_commitments.iter().all(|v| non_empty(v))
            && non_empty(&self.excluded_boundary_policy)
            && self.max_nodes > 0 && self.max_edges > 0
            && self.claim_ceiling == D6X_CLAIM_CEILING
    }
    pub fn commitment(&self) -> String { canonical_sha256("d6x-closure-profile", self) }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SemanticDependencyKindV1 {
    Node,
    Edge,
    D6PReceipt,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SemanticDependencyContextV1 {
    Edge {
        from_node_id: String,
        to_node_id: String,
        kind: ClaimGraphEdgeKindV1,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct SemanticDependencyReferenceV1 {
    pub kind: SemanticDependencyKindV1,
    pub identifier: String,
    pub commitment: Option<String>,
    /// Edge context is part of semantic identity; node/D6P references have no context.
    pub context: Option<SemanticDependencyContextV1>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SemanticDependencyResolutionV1 {
    Present,
    Missing,
    Stale,
}

/// Runtime/audit evidence about resolving a semantic dependency.
/// This is deliberately excluded from closure identity: retrieval addresses and
/// observed evidence may vary without changing what the semantic dependency is.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticDependencyResolutionEvidenceV1 {
    pub retrieval_reference: Option<String>,
    pub observed_commitment: Option<String>,
    pub qualification_context_commitment: Option<String>,
}

impl SemanticDependencyResolutionEvidenceV1 {
    pub fn structurally_valid(&self) -> bool {
        [
            self.retrieval_reference.as_deref(),
            self.observed_commitment.as_deref(),
            self.qualification_context_commitment.as_deref(),
        ]
        .into_iter()
        .flatten()
        .next()
        .is_some()
        && [
            self.retrieval_reference.as_deref(),
            self.observed_commitment.as_deref(),
            self.qualification_context_commitment.as_deref(),
        ]
        .into_iter()
        .flatten()
        .all(non_empty)
    }
}

impl SemanticDependencyReferenceV1 {
    pub fn node(id: impl Into<String>, commitment: Option<String>) -> Self {
        Self { kind: SemanticDependencyKindV1::Node, identifier: id.into(), commitment, context: None }
    }

    pub fn edge(
        id: impl Into<String>,
        from_node_id: impl Into<String>,
        to_node_id: impl Into<String>,
        kind: ClaimGraphEdgeKindV1,
        commitment: Option<String>,
    ) -> Self {
        Self {
            kind: SemanticDependencyKindV1::Edge,
            identifier: id.into(),
            commitment,
            context: Some(SemanticDependencyContextV1::Edge {
                from_node_id: from_node_id.into(),
                to_node_id: to_node_id.into(),
                kind,
            }),
        }
    }

    pub fn d6p_receipt(commitment: impl Into<String>) -> Self {
        let commitment = commitment.into();
        Self {
            kind: SemanticDependencyKindV1::D6PReceipt,
            identifier: commitment.clone(),
            commitment: Some(commitment),
            context: None,
        }
    }

    pub fn structurally_valid(&self) -> bool {
        if !non_empty(&self.identifier) || self.commitment.as_deref().map_or(false, |v| !non_empty(v)) {
            return false;
        }
        match (&self.kind, &self.context, &self.commitment) {
            (SemanticDependencyKindV1::Node, None, _) => true,
            (SemanticDependencyKindV1::Edge, Some(SemanticDependencyContextV1::Edge { from_node_id, to_node_id, .. }), _) =>
                non_empty(from_node_id) && non_empty(to_node_id) && from_node_id != to_node_id,
            (SemanticDependencyKindV1::D6PReceipt, None, Some(commitment)) =>
                self.identifier == *commitment,
            _ => false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DependencyClosureCertificateV1 {
    pub schema_version: String,
    pub algorithm_version: String,
    pub closure_profile_commitment: String,
    pub source_dkg_snapshot_commitment: String,
    pub projection_commitment: String,
    /// Candidate-independent identity of the selected semantic closure.
    pub closure_identity_commitment: String,
    pub semantic_environment_commitment: String,
    pub derivation_profile_commitment: String,
    pub root_node_ids: BTreeSet<String>,
    pub included_node_commitments: BTreeSet<String>,
    pub included_node_ids: BTreeSet<String>,
    /// Exact selected node identity: node id -> node commitment.
    /// This pairing is part of the candidate-independent closure identity.
    pub included_nodes: BTreeMap<String, String>,
    pub included_d6p_receipt_commitments: BTreeSet<String>,
    pub included_edge_commitments: BTreeSet<String>,
    pub included_edges: BTreeMap<String, (String, String, ClaimGraphEdgeKindV1, String)>,
    /// Legacy flat IDs retained for wire compatibility; typed dependencies are authoritative for identity.
    pub missing_dependency_ids: BTreeSet<String>,
    pub missing_dependencies: BTreeSet<SemanticDependencyReferenceV1>,
    /// Canonical typed set of all selected semantic dependencies.
    pub dependencies: BTreeSet<SemanticDependencyReferenceV1>,
    /// Per-dependency resolution state. The key set must exactly cover selected
    /// and missing dependencies; "not selected" is represented by absence.
    pub dependency_resolutions: BTreeMap<SemanticDependencyReferenceV1, SemanticDependencyResolutionV1>,
    /// Optional audit/retrieval evidence. Never part of semantic closure identity.
    pub resolution_evidence: BTreeMap<SemanticDependencyReferenceV1, SemanticDependencyResolutionEvidenceV1>,
    pub status: DependencyClosureStatusV1,
    pub cycle_detected: bool,
    pub claim_ceiling: String,
    pub commitment: String,
}
impl DependencyClosureCertificateV1 {
    pub fn valid(&self) -> bool {
        let expected_dependencies = self.expected_dependencies();
        let expected_missing_ids = self
            .missing_dependencies
            .iter()
            .map(|dependency| dependency.identifier.clone())
            .collect::<BTreeSet<_>>();

        self.schema_version == D6X_SCHEMA_VERSION
            && self.algorithm_version == D6X_ALGORITHM_VERSION
            && non_empty(&self.closure_profile_commitment)
            && non_empty(&self.source_dkg_snapshot_commitment)
            && non_empty(&self.projection_commitment)
            && non_empty(&self.closure_identity_commitment)
            && non_empty(&self.semantic_environment_commitment)
            && non_empty(&self.derivation_profile_commitment)
            && !self.root_node_ids.is_empty()
            && self.included_node_commitments.iter().all(|v| non_empty(v))
            && self.included_node_ids.iter().all(|v| non_empty(v))
            && self.included_nodes.iter().all(|(id, commitment)| non_empty(id) && non_empty(commitment))
            && self.included_nodes.keys().cloned().collect::<BTreeSet<_>>() == self.included_node_ids
            && self.included_nodes.values().cloned().collect::<BTreeSet<_>>() == self.included_node_commitments
            && self.included_d6p_receipt_commitments.iter().all(|v| non_empty(v))
            && self.included_edge_commitments.iter().all(|v| non_empty(v))
            && self.included_edges.iter().all(|(id, (from, to, _kind, commitment))|
                non_empty(id) && non_empty(from) && non_empty(to) && from != to && non_empty(commitment))
            && self.missing_dependency_ids.iter().all(|v| non_empty(v))
            && self.missing_dependencies.iter().all(SemanticDependencyReferenceV1::structurally_valid)
            && self.dependencies.iter().all(SemanticDependencyReferenceV1::structurally_valid)
            && self.dependencies == expected_dependencies
            && self.missing_dependency_ids == expected_missing_ids
            && self.dependency_resolutions.keys().cloned().collect::<BTreeSet<_>>() == self.dependencies.union(&self.missing_dependencies).cloned().collect()
            && self.dependencies.iter().all(|dependency| matches!(self.dependency_resolutions.get(dependency), Some(SemanticDependencyResolutionV1::Present | SemanticDependencyResolutionV1::Stale)))
            && self.missing_dependencies.iter().all(|dependency| matches!(self.dependency_resolutions.get(dependency), Some(SemanticDependencyResolutionV1::Missing)))
            && self.resolution_evidence.iter().all(|(dependency, evidence)| {
                self.resolution_evidence_is_consistent(dependency, evidence)
            })
            && self.claim_ceiling == D6X_CLAIM_CEILING
            && self.included_node_ids.len() == self.included_node_commitments.len()
            && self.included_edges.len() == self.included_edge_commitments.len()
            && self.included_edges.values().all(|(from, to, _kind, commitment)|
                self.included_node_ids.contains(from) && self.included_node_ids.contains(to)
                    && self.included_edge_commitments.contains(commitment))
            && self.closure_identity_commitment == self.closure_identity()
            && {
                let has_stale = self.dependencies.iter().any(|dependency| {
                    self.dependency_resolutions.get(dependency) == Some(&SemanticDependencyResolutionV1::Stale)
                });
                match self.status {
                    DependencyClosureStatusV1::Complete =>
                        self.missing_dependency_ids.is_empty() && !has_stale,
                    DependencyClosureStatusV1::BlockedMissingDependency =>
                        !self.missing_dependency_ids.is_empty(),
                    DependencyClosureStatusV1::BlockedCurrentness =>
                        has_stale,
                    DependencyClosureStatusV1::BlockedResourceLimit =>
                        !has_stale,
                }
            }
            && self.commitment == self.recompute()
    }

    fn resolution_evidence_is_consistent(
        &self,
        dependency: &SemanticDependencyReferenceV1,
        evidence: &SemanticDependencyResolutionEvidenceV1,
    ) -> bool {
        if !evidence.structurally_valid() {
            return false;
        }

        let resolution = match self.dependency_resolutions.get(dependency) {
            Some(resolution) => resolution,
            None => return false,
        };

        match resolution {
            SemanticDependencyResolutionV1::Missing => {
                // A missing dependency has no observed semantic content. The
                // retrieval reference may still document where resolution was
                // attempted, but it must not claim an observed commitment.
                evidence.observed_commitment.is_none()
            }
            SemanticDependencyResolutionV1::Present | SemanticDependencyResolutionV1::Stale => {
                match (&dependency.commitment, &evidence.observed_commitment) {
                    (Some(expected), Some(observed)) => expected == observed,
                    // Omitting an observed commitment is allowed when the
                    // runtime cannot expose it, but a supplied observation must
                    // agree with the semantic dependency commitment.
                    _ => true,
                }
            }
        }
    }

    fn expected_dependencies(&self) -> BTreeSet<SemanticDependencyReferenceV1> {
        let mut expected = BTreeSet::new();
        expected.extend(
            self.included_nodes
                .iter()
                .map(|(id, commitment)| SemanticDependencyReferenceV1::node(id.clone(), Some(commitment.clone()))),
        );
        expected.extend(
            self.included_edges.iter().map(|(id, (from, to, kind, commitment))|
                SemanticDependencyReferenceV1::edge(
                    id.clone(),
                    from.clone(),
                    to.clone(),
                    *kind,
                    Some(commitment.clone()),
                )
            ),
        );
        expected.extend(
            self.included_d6p_receipt_commitments
                .iter()
                .cloned()
                .map(SemanticDependencyReferenceV1::d6p_receipt),
        );
        expected
    }
    pub fn closure_identity(&self) -> String {
        let identity = (
            &self.schema_version,
            &self.algorithm_version,
            &D6X_CANONICALIZATION_VERSION,
            &self.closure_profile_commitment,
            &self.source_dkg_snapshot_commitment,
            &self.semantic_environment_commitment,
            &self.derivation_profile_commitment,
            &self.root_node_ids,
            &self.dependencies,
            &self.missing_dependencies,
            &self.dependency_resolutions,
            &self.status,
            &self.cycle_detected,
            &self.claim_ceiling,
        );
        canonical_sha256("d6x-closure-identity", &identity)
    }

    pub fn recompute(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.commitment.clear();
        canonical_sha256("d6x-dependency-closure", &unsigned)
    }
}

fn cycle_exists(nodes: &BTreeSet<String>, edges: &[(String, String)]) -> bool {
    fn visit(id: &str, adjacency: &BTreeMap<String, BTreeSet<String>>, active: &mut BTreeSet<String>, done: &mut BTreeSet<String>) -> bool {
        if active.contains(id) { return true; }
        if done.contains(id) { return false; }
        active.insert(id.to_owned());
        if adjacency.get(id).is_some_and(|next| next.iter().any(|child| visit(child, adjacency, active, done))) { return true; }
        active.remove(id); done.insert(id.to_owned()); false
    }
    let mut adjacency: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for (from, to) in edges {
        if nodes.contains(from) && nodes.contains(to) { adjacency.entry(from.clone()).or_default().insert(to.clone()); }
    }
    let mut active = BTreeSet::new();
    let mut done = BTreeSet::new();
    adjacency.keys().any(|id| visit(id, &adjacency, &mut active, &mut done))
}

pub fn compute_dependency_closure(
    projection: &QualifiedProjectionV1,
    environment: &SemanticEnvironmentV1,
    derivation_profile: &DerivationProfileV1,
    profile: &DependencyClosureProfileV1,
) -> Option<DependencyClosureCertificateV1> {
    if !projection.structurally_valid() || !environment.structurally_valid()
        || !derivation_profile.structurally_valid() || !profile.structurally_valid() { return None; }
    // The closure boundary must not accept a projection whose node/edge,
    // environment, or derivation commitments no longer match their semantic
    // sources. This also makes the source-snapshot identity an explicit input
    // to the D6X boundary rather than an unchecked opaque field.
    if !projection.commitments_match_sources(environment, derivation_profile) { return None; }
    if projection.semantic_environment_commitment != environment.commitment()
        || projection.derivation_profile_commitment != derivation_profile.commitment() { return None; }

    let mut included_ids = BTreeSet::new();
    let mut included_edges = BTreeSet::new();
    let mut missing = BTreeSet::new();
    let mut missing_dependencies = BTreeSet::new();
    let mut dependencies = BTreeSet::new();
    let mut dependency_resolutions: BTreeMap<SemanticDependencyReferenceV1, SemanticDependencyResolutionV1> = BTreeMap::new();
    let selected_d6p_receipts: BTreeSet<String> = profile.required_d6p_receipt_commitments.intersection(&projection.d6p_current_receipt_commitments).cloned().collect();
    let missing_d6p_receipts: BTreeSet<String> = profile.required_d6p_receipt_commitments.difference(&projection.d6p_current_receipt_commitments).cloned().collect();
    for receipt in &selected_d6p_receipts {
        let dependency = SemanticDependencyReferenceV1::d6p_receipt(receipt.clone());
        dependencies.insert(dependency.clone());
        dependency_resolutions.insert(dependency, SemanticDependencyResolutionV1::Present);
    }
    for receipt in missing_d6p_receipts {
        missing.insert(receipt.clone());
        let dependency = SemanticDependencyReferenceV1::d6p_receipt(receipt);
        missing_dependencies.insert(dependency.clone());
        dependency_resolutions.insert(dependency, SemanticDependencyResolutionV1::Missing);
    }
    let mut blocked_currentness = false;
    let mut resource_blocked = false;
    let mut queue = VecDeque::from_iter(profile.root_node_ids.iter().cloned());
    for id in &profile.required_node_ids {
        if projection.nodes.contains_key(id) { queue.push_back(id.clone()); }
    }
    let mut queued = BTreeSet::new();
    for id in profile.root_node_ids.iter().chain(profile.required_node_ids.iter()) {
        if !projection.nodes.contains_key(id) {
            missing.insert(id.clone());
            let dependency = SemanticDependencyReferenceV1::node(id.clone(), None);
            missing_dependencies.insert(dependency.clone());
            dependency_resolutions.insert(dependency, SemanticDependencyResolutionV1::Missing);
        }
    }

    while let Some(id) = queue.pop_front() {
        if !queued.insert(id.clone()) { continue; }
        let Some(node) = projection.nodes.get(&id) else {
            let dependency = SemanticDependencyReferenceV1::node(id.clone(), None);
            missing_dependencies.insert(dependency.clone());
            dependency_resolutions.insert(dependency, SemanticDependencyResolutionV1::Missing);
            missing.insert(id);
            continue;
        };
        // A selected node commitment is a cryptographic binding, not merely
        // an opaque identifier. Reject stale (semantic fields, commitment)
        // pairs before they can enter the closure identity.
        if !node.commitment_matches() {
            return None;
        }
        if included_ids.len() as u32 >= profile.max_nodes {
            resource_blocked = true;
            break;
        }
        included_ids.insert(id.clone());
        let node_dependency = SemanticDependencyReferenceV1::node(id.clone(), Some(node.node_commitment.clone()));
        dependencies.insert(node_dependency.clone());
        dependency_resolutions.insert(node_dependency.clone(), SemanticDependencyResolutionV1::Present);

        for edge in projection.edges.values() {
            if edge.from_node_id != id { continue; }

            let (matches_rule, requires_current) = match projection.nodes.get(&edge.to_node_id) {
                Some(to) => (
                    profile.rules.iter().any(|r| r.matches(node.kind, to.kind, edge.kind)),
                    profile.rules.iter().any(|r|
                        r.matches(node.kind, to.kind, edge.kind)
                            && r.currentness == DependencyCurrentnessV1::CurrentOnly
                    ),
                ),
                None => {
                    // A missing target has no node kind, so only an explicitly
                    // wildcard-target rule can qualify the dangling relationship.
                    (
                        profile.rules.iter().any(|r| {
                            r.edge_kind == edge.kind
                                && r.from_kind.map_or(true, |kind| kind == node.kind)
                                && r.to_kind.is_none()
                        }),
                        profile.rules.iter().any(|r| {
                            r.edge_kind == edge.kind
                                && r.from_kind.map_or(true, |kind| kind == node.kind)
                                && r.to_kind.is_none()
                                && r.currentness == DependencyCurrentnessV1::CurrentOnly
                        }),
                    )
                }
            };

            if !matches_rule { continue; }
            // Only selected semantic edges are required to prove their own
            // commitment binding. Irrelevant/provenance candidates remain
            // outside the semantic closure boundary.
            if !edge.commitment_matches() {
                return None;
            }
            // Overlapping rules resolve monotonically: CurrentOnly dominates Any.
            let Some(to) = projection.nodes.get(&edge.to_node_id) else {
                let dependency = SemanticDependencyReferenceV1::node(edge.to_node_id.clone(), None);
                missing_dependencies.insert(dependency.clone());
                dependency_resolutions.insert(dependency, SemanticDependencyResolutionV1::Missing);
                missing.insert(edge.to_node_id.clone());
                continue;
            };
            if included_edges.len() as u32 >= profile.max_edges {
                resource_blocked = true;
                break;
            }
            included_edges.insert(edge.edge_id.clone());
            let edge_dependency = SemanticDependencyReferenceV1::edge(
                edge.edge_id.clone(),
                edge.from_node_id.clone(),
                edge.to_node_id.clone(),
                edge.kind,
                Some(edge.edge_commitment.clone()),
            );
            dependencies.insert(edge_dependency.clone());
            dependency_resolutions.insert(edge_dependency, SemanticDependencyResolutionV1::Present);
            if requires_current && to.historical_only {
                blocked_currentness = true;
                let target_dependency = SemanticDependencyReferenceV1::node(
                    to.node_id.clone(),
                    Some(to.node_commitment.clone()),
                );
                dependency_resolutions
                    .entry(target_dependency)
                    .and_modify(|resolution| {
                        if *resolution != SemanticDependencyResolutionV1::Stale {
                            *resolution = SemanticDependencyResolutionV1::Stale;
                        }
                    })
                    .or_insert(SemanticDependencyResolutionV1::Stale);
            }
            if !queued.contains(&to.node_id) { queue.push_back(to.node_id.clone()); }
        }
    }

    let selected_edges: Vec<_> = included_edges.iter().filter_map(|id|
        projection.edges.get(id).map(|e| (e.from_node_id.clone(), e.to_node_id.clone()))
    ).collect();
    let cycle_detected = cycle_exists(&included_ids, &selected_edges);
    let status = if !missing.is_empty() {
        DependencyClosureStatusV1::BlockedMissingDependency
    } else if blocked_currentness {
        DependencyClosureStatusV1::BlockedCurrentness
    } else if resource_blocked {
        DependencyClosureStatusV1::BlockedResourceLimit
    } else {
        DependencyClosureStatusV1::Complete
    };

    let mut out = DependencyClosureCertificateV1 {
        schema_version: D6X_SCHEMA_VERSION.into(),
        algorithm_version: D6X_ALGORITHM_VERSION.into(),
        closure_profile_commitment: profile.commitment(),
        source_dkg_snapshot_commitment: projection.source_dkg_snapshot_commitment.clone(),
        projection_commitment: projection.commitment(),
        closure_identity_commitment: String::new(),
        semantic_environment_commitment: environment.commitment(),
        derivation_profile_commitment: derivation_profile.commitment(),
        root_node_ids: profile.root_node_ids.clone(),
        included_node_commitments: included_ids.iter().filter_map(|id| projection.nodes.get(id).map(|n| n.node_commitment.clone())).collect(),
        included_node_ids: included_ids.clone(),
        included_nodes: included_ids.iter().filter_map(|id| projection.nodes.get(id).map(|n| (id.clone(), n.node_commitment.clone()))).collect(),
        included_d6p_receipt_commitments: selected_d6p_receipts,
        included_edge_commitments: included_edges.iter().filter_map(|id| projection.edges.get(id).map(|e| e.edge_commitment.clone())).collect(),
        included_edges: included_edges.iter().filter_map(|id| projection.edges.get(id).map(|e| (id.clone(), (e.from_node_id.clone(), e.to_node_id.clone(), e.kind, e.edge_commitment.clone())))).collect(),
        missing_dependency_ids: missing,
        missing_dependencies,
        dependencies,
        dependency_resolutions,
        resolution_evidence: BTreeMap::new(),
        status, cycle_detected, claim_ceiling: D6X_CLAIM_CEILING.into(), commitment: String::new(),
    };
    out.closure_identity_commitment = out.closure_identity();
    out.commitment = out.recompute();
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::canonical_derivation_receipt::{QualifiedEdgeV1, QualifiedNodeV1};

    fn profile(required: BTreeSet<String>) -> DependencyClosureProfileV1 {
        let rule = DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::Any,
        };
        DependencyClosureProfileV1 {
            profile_id:"closure".into(), version:"1".into(),
            root_node_ids:["root".into()].into_iter().collect(),
            required_node_ids:required, required_d6p_receipt_commitments:BTreeSet::new(), rules:[rule].into_iter().collect(),
            excluded_boundary_policy:"Only rule-matched semantic edges expand closure.".into(),
            max_nodes:16, max_edges:16, claim_ceiling:D6X_CLAIM_CEILING.into(),
        }
    }

    fn projection(extra: bool) -> (QualifiedProjectionV1, SemanticEnvironmentV1, DerivationProfileV1) {
        let env = SemanticEnvironmentV1 {
            semantic_profile_id:"sem".into(), semantic_profile_version:"1".into(),
            current_frontier_root:Some("frontier".into()), d6p_eligibility_context_root:None,
            d6n_observer_context_root:None, d6o_lifecycle_context_root:None,
            membership_authority_scope_root:None, dependency_snapshot_root:Some("snapshot".into()),
            historical_cutoff:None, policy_version:"policy-1".into(), claim_ceiling:D6S_CLAIM_CEILING.into(),
        };
        let dp = DerivationProfileV1 {
            profile_id:"derivation".into(), version:"1".into(),
            rule_ids:["rule".into()].into_iter().collect(), permits_recursive_fixpoint:false,
            claim_ceiling:D6S_CLAIM_CEILING.into(),
        };
        let mut nodes = BTreeMap::new();
        for (id, kind) in [("root", ClaimGraphNodeKindV1::Statement),("dep", ClaimGraphNodeKindV1::Evidence)] {
            nodes.insert(id.into(), QualifiedNodeV1 {
                node_id:id.into(), kind, content_commitment:format!("content-{id}"), node_commitment:format!("commit-{id}"),
                historical_only:false, current_frontier_root:Some("frontier".into()), claim_ceiling:D6S_CLAIM_CEILING.into(),
            });
        }
        if extra {
            nodes.insert("noise".into(), QualifiedNodeV1 {
                node_id:"noise".into(), kind:ClaimGraphNodeKindV1::Source, content_commitment:"content-noise".into(), node_commitment:"commit-noise".into(),
                historical_only:false, current_frontier_root:Some("frontier".into()), claim_ceiling:D6S_CLAIM_CEILING.into(),
            });
        }
        let mut edges = BTreeMap::new();
        edges.insert("e1".into(), QualifiedEdgeV1 {
            edge_id:"e1".into(), from_node_id:"root".into(), to_node_id:"dep".into(),
            kind:ClaimGraphEdgeKindV1::Supports, edge_commitment:"edge-e1".into(), claim_ceiling:D6S_CLAIM_CEILING.into(),
        });
        if extra {
            edges.insert("noise-edge".into(), QualifiedEdgeV1 {
                edge_id:"noise-edge".into(), from_node_id:"noise".into(), to_node_id:"dep".into(),
                kind:ClaimGraphEdgeKindV1::Provenance, edge_commitment:"edge-noise".into(), claim_ceiling:D6S_CLAIM_CEILING.into(),
            });
        }
        let p = QualifiedProjectionV1 {
            projection_id:"projection".into(), projection_version:"1".into(),
            canonicalization_version:"D6S-CANON-1".into(), source_dkg_snapshot_commitment:"snapshot".into(),
            nodes, edges, d6p_current_receipt_commitments:BTreeSet::new(), d6n_context_commitment:None,
            d6o_context_commitment:None, semantic_environment_commitment:env.commitment(),
            derivation_profile_commitment:dp.commitment(), claim_ceiling:D6S_CLAIM_CEILING.into(),
        };
        (p, env, dp)
    }

    #[test]
    fn irrelevant_dkg_material_does_not_change_closure() {
        let (a,e,d)=projection(false); let (b,_,_)=projection(true); let p=profile(BTreeSet::new());
        let ca=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        let cb=compute_dependency_closure(&b,&e,&d,&p).unwrap();
        assert_ne!(ca.commitment, cb.commitment); // audit/provenance certificate remains bound to candidate projection
        assert_eq!(ca.closure_identity_commitment, cb.closure_identity_commitment);
        assert_eq!(ca.included_nodes, cb.included_nodes);
    }

    #[test]
    fn missing_required_dependency_blocks_closure() {
        let (a,e,d)=projection(false); let p=profile(["missing".into()].into_iter().collect());
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_eq!(c.status, DependencyClosureStatusV1::BlockedMissingDependency);
        assert!(c.missing_dependency_ids.contains("missing")); assert!(c.valid());
    }

    #[test]
    fn provenance_edge_alone_does_not_expand_closure() {
        let (a,e,d)=projection(true); let p=profile(BTreeSet::new());
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert!(!c.included_node_commitments.contains("commit-noise"));
        assert_eq!(c.included_node_ids, ["root".to_string(), "dep".to_string()].into_iter().collect());
    }

    #[test]
    fn disconnected_required_node_is_included() {
        let (a,e,d)=projection(false); let p=profile(["dep".into()].into_iter().collect());
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_eq!(c.status, DependencyClosureStatusV1::Complete);
        assert!(c.included_node_ids.contains("dep"));
    }

    #[test]
    fn exact_resource_limit_is_not_blocked_when_no_work_remains() {
        let (a,e,d)=projection(false); let mut p=profile(BTreeSet::new());
        p.max_nodes=2; p.max_edges=1;
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_eq!(c.status, DependencyClosureStatusV1::Complete);
    }

    #[test]
    fn source_snapshot_mutation_changes_d6x_identity() {
        let (baseline, environment, derivation_profile) = projection(false);
        let before = compute_dependency_closure(
            &baseline,
            &environment,
            &derivation_profile,
            &profile(BTreeSet::new()),
        )
        .unwrap();

        let mut changed = baseline.clone();
        changed.source_dkg_snapshot_commitment = "different-snapshot".into();
        let after = compute_dependency_closure(
            &changed,
            &environment,
            &derivation_profile,
            &profile(BTreeSet::new()),
        )
        .unwrap();

        assert_ne!(before.source_dkg_snapshot_commitment, after.source_dkg_snapshot_commitment);
        assert_ne!(before.closure_identity_commitment, after.closure_identity_commitment);
    }

    #[test]
    fn selected_node_commitment_changes_identity() {
        let (mut a,e,d)=projection(false); let p=profile(BTreeSet::new());
        let before=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        { let dep=a.nodes.get_mut("dep").unwrap(); dep.content_commitment="changed".into(); dep.node_commitment=dep.recomputed_commitment(); }
        let after=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_ne!(before.closure_identity_commitment, after.closure_identity_commitment);
    }

    #[test]
    fn selected_edge_commitment_changes_identity() {
        let (mut a,e,d)=projection(false); let p=profile(BTreeSet::new());
        let before=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        { let edge=a.edges.get_mut("e1").unwrap(); edge.kind=ClaimGraphEdgeKindV1::DerivedFrom; edge.edge_commitment=edge.recomputed_commitment(); }
        let after=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_ne!(before.closure_identity_commitment, after.closure_identity_commitment);
    }

    #[test]
    fn irrelevant_node_does_not_change_identity() {
        let (a,e,d)=projection(false); let (b,_,_)=projection(true); let p=profile(BTreeSet::new());
        assert_eq!(compute_dependency_closure(&a,&e,&d,&p).unwrap().closure_identity_commitment,
                   compute_dependency_closure(&b,&e,&d,&p).unwrap().closure_identity_commitment);
    }

    #[test]
    fn selected_edge_kind_changes_identity() {
        let (mut a,e,d)=projection(false); let p=profile(BTreeSet::new());
        let before=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        {
            let edge = a.edges.get_mut("e1").unwrap();
            edge.kind=ClaimGraphEdgeKindV1::Provenance;
            edge.edge_commitment = edge.recomputed_commitment();
        }
        let after=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_ne!(before.closure_identity_commitment, after.closure_identity_commitment);
    }

    #[test]
    fn closure_profile_change_changes_identity() {
        let (a,e,d)=projection(false); let p=profile(BTreeSet::new());
        let before=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        let mut changed=p.clone();
        changed.version="2".into();
        let after=compute_dependency_closure(&a,&e,&d,&changed).unwrap();
        assert_ne!(before.closure_identity_commitment, after.closure_identity_commitment);
    }

    #[test]
    fn set_insertion_order_does_not_change_identity() {
        let (a,e,d)=projection(false);
        let mut p1=profile(BTreeSet::new());
        p1.required_node_ids.extend(["dep".into(), "root".into()]);
        let mut p2=p1.clone();
        p2.required_node_ids=BTreeSet::from(["root".into(), "dep".into()]);
        assert_eq!(compute_dependency_closure(&a,&e,&d,&p1).unwrap().closure_identity_commitment,
                   compute_dependency_closure(&a,&e,&d,&p2).unwrap().closure_identity_commitment);
    }

    #[test]
    fn canonical_dependency_set_contains_selected_node_and_edge() {
        let (a,e,d)=projection(false); let p=profile(BTreeSet::new());
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert!(c.dependencies.contains(&SemanticDependencyReferenceV1::node("root", Some("commit-root".into()))));
        assert!(c.dependencies.contains(&SemanticDependencyReferenceV1::node("dep", Some("commit-dep".into()))));
        assert!(c.dependencies.contains(&SemanticDependencyReferenceV1::edge("e1", "root", "dep", ClaimGraphEdgeKindV1::Supports, Some("edge-e1".into()))));
        assert_eq!(c.dependencies.len(), 3);
    }

    #[test]
    fn selected_edge_endpoint_or_kind_changes_identity() {
        let (mut a,e,d)=projection(false); let p=profile(BTreeSet::new());
        let before=compute_dependency_closure(&a,&e,&d,&p).unwrap();

        a.nodes.insert("alt".into(), QualifiedNodeV1 {
            node_id:"alt".into(), kind:ClaimGraphNodeKindV1::Evidence,
            content_commitment:"content-alt".into(), node_commitment:"commit-alt".into(), historical_only:false,
            current_frontier_root:Some("frontier".into()), claim_ceiling:D6S_CLAIM_CEILING.into(),
        });
        {
            let edge = a.edges.get_mut("e1").unwrap();
            edge.to_node_id = "alt".into();
            edge.edge_commitment = edge.recomputed_commitment();
        }
        let endpoint_changed=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_ne!(before.closure_identity_commitment, endpoint_changed.closure_identity_commitment);

        let (mut a,e,d)=projection(false);
        let before=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        {
            let edge = a.edges.get_mut("e1").unwrap();
            edge.kind = ClaimGraphEdgeKindV1::Provenance;
            edge.edge_commitment = edge.recomputed_commitment();
        }
        let kind_changed=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_ne!(before.closure_identity_commitment, kind_changed.closure_identity_commitment);
    }

    #[test]
    fn dependency_reference_domains_are_structurally_distinct() {
        let node = SemanticDependencyReferenceV1::node("x", Some("node-c".into()));
        let edge = SemanticDependencyReferenceV1::edge(
            "x", "a", "b", ClaimGraphEdgeKindV1::Supports, Some("edge-c".into())
        );
        let d6p = SemanticDependencyReferenceV1::d6p_receipt("receipt-c");
        assert!(node.structurally_valid());
        assert!(edge.structurally_valid());
        assert!(d6p.structurally_valid());

        let invalid_d6p = SemanticDependencyReferenceV1 {
            kind: SemanticDependencyKindV1::D6PReceipt,
            identifier: "receipt-c".into(),
            commitment: Some("different".into()),
            context: None,
        };
        assert!(!invalid_d6p.structurally_valid());

        let invalid_edge = SemanticDependencyReferenceV1 {
            kind: SemanticDependencyKindV1::Edge,
            identifier: "x".into(),
            commitment: Some("edge-c".into()),
            context: None,
        };
        assert!(!invalid_edge.structurally_valid());
    }

    #[test]
    fn currentness_marks_the_affected_dependency_stale() {
        let (mut a,e,d)=projection(false);
        a.nodes.get_mut("dep").unwrap().historical_only = true;
        let mut p=profile(BTreeSet::new());
        p.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::CurrentOnly,
        }].into_iter().collect();
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        let dep=SemanticDependencyReferenceV1::node("dep", Some("commit-dep".into()));
        assert_eq!(c.status, DependencyClosureStatusV1::BlockedCurrentness);
        assert!(c.dependencies.contains(&SemanticDependencyReferenceV1::edge(
            "e1", "root", "dep", ClaimGraphEdgeKindV1::Supports, Some("edge-e1".into())
        )));
        assert_eq!(c.dependency_resolutions.get(&dep), Some(&SemanticDependencyResolutionV1::Stale));
        assert!(c.valid());
    }

    #[test]
    fn stale_resolution_is_monotonic_across_selected_edge_order() {
        let (mut a,e,d)=projection(false);
        a.nodes.get_mut("dep").unwrap().historical_only = true;
        a.edges.insert("z-current".into(), QualifiedEdgeV1 {
            edge_id:"z-current".into(), from_node_id:"root".into(), to_node_id:"dep".into(),
            kind:ClaimGraphEdgeKindV1::Supports, edge_commitment:"edge-z".into(),
            claim_ceiling:D6X_CLAIM_CEILING.into(),
        });
        let mut p=profile(BTreeSet::new());
        p.rules = [
            DependencyRuleV1 {
                edge_kind: ClaimGraphEdgeKindV1::Supports,
                from_kind: Some(ClaimGraphNodeKindV1::Statement),
                to_kind: Some(ClaimGraphNodeKindV1::Evidence),
                currentness: DependencyCurrentnessV1::Any,
            },
            DependencyRuleV1 {
                edge_kind: ClaimGraphEdgeKindV1::Supports,
                from_kind: Some(ClaimGraphNodeKindV1::Statement),
                to_kind: Some(ClaimGraphNodeKindV1::Evidence),
                currentness: DependencyCurrentnessV1::CurrentOnly,
            },
        ].into_iter().collect();
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        let dep=SemanticDependencyReferenceV1::node("dep", Some("commit-dep".into()));
        assert_eq!(c.dependency_resolutions.get(&dep), Some(&SemanticDependencyResolutionV1::Stale));
        assert_eq!(c.status, DependencyClosureStatusV1::BlockedCurrentness);
    }

    #[test]
    fn stale_resolution_requires_a_selected_current_only_edge() {
        let (mut a,e,d)=projection(false);
        a.nodes.insert("other".into(), QualifiedNodeV1 {
            node_id:"other".into(), kind:ClaimGraphNodeKindV1::Statement,
            content_commitment:"content-other".into(), node_commitment:"commit-other".into(), historical_only:false,
            current_frontier_root:Some("frontier".into()), claim_ceiling:D6S_CLAIM_CEILING.into(),
        });
        a.edges.insert("unselected-current-only".into(), QualifiedEdgeV1 {
            edge_id:"unselected-current-only".into(),
            from_node_id:"other".into(), to_node_id:"dep".into(),
            kind:ClaimGraphEdgeKindV1::Supports, edge_commitment:"edge-unselected".into(),
            claim_ceiling:D6S_CLAIM_CEILING.into(),
        });
        a.nodes.get_mut("dep").unwrap().historical_only = true;
        let mut p=profile(BTreeSet::new());
        p.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::Any,
        }].into_iter().collect();
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        let dep=SemanticDependencyReferenceV1::node("dep", Some("commit-dep".into()));
        assert_eq!(c.dependency_resolutions.get(&dep), Some(&SemanticDependencyResolutionV1::Present));
        assert_eq!(c.status, DependencyClosureStatusV1::Complete);
    }

    #[test]
    fn missing_unmatched_edge_target_does_not_block_closure() {
        let (mut a,e,d)=projection(false);
        a.edges.insert("irrelevant-dangling".into(), QualifiedEdgeV1 {
            edge_id:"irrelevant-dangling".into(),
            from_node_id:"root".into(), to_node_id:"missing-target".into(),
            kind:ClaimGraphEdgeKindV1::Provenance, edge_commitment:"edge-dangling".into(),
            claim_ceiling:D6S_CLAIM_CEILING.into(),
        });
        let p=profile(BTreeSet::new());
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_eq!(c.status, DependencyClosureStatusV1::Complete);
        assert!(!c.missing_dependency_ids.contains("missing-target"));
    }

    #[test]
    fn dependency_resolution_distinguishes_present_and_missing() {
        let (a,e,d)=projection(false); let p=profile(["missing".into()].into_iter().collect());
        let c=compute_dependency_closure(&a,&e,&d,&p).unwrap();
        assert_eq!(c.dependency_resolutions.get(&SemanticDependencyReferenceV1::node("root", Some("commit-root".into()))), Some(&SemanticDependencyResolutionV1::Present));
        assert_eq!(c.dependency_resolutions.get(&SemanticDependencyReferenceV1::node("missing", None)), Some(&SemanticDependencyResolutionV1::Missing));
        assert!(c.valid());