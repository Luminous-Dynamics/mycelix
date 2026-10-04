//! D6X qualified semantic dependency closure reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! D6X is narrower than graph reachability. A closure is the deterministic
//! result of applying a named dependency policy to a qualified projection.
//! Provenance/custody and unrelated graph material are not dependencies unless
//! the profile explicitly makes them dependencies.
use crate::canonical_derivation_receipt::{
    canonical_sha256, is_canonical_sha256_commitment, DerivationProfileV1, QualifiedProjectionV1, SemanticEnvironmentV1,
    D6S_CLAIM_CEILING,
};
use crate::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
use crate::finality_eligibility_composition::{verify_current_receipt_provenance_from_composition, CurrentFinalityEligibilityReceiptV1, FinalityEligibilityCompositionV1};
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
            && self.root_node_ids.iter().all(|id| self.included_node_ids.contains(id))
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
                    // Status precedence is deterministic and mirrors the
                    // constructor: missing dependencies dominate currentness,
                    // and currentness dominates resource truncation.
                    DependencyClosureStatusV1::BlockedMissingDependency =>
                        !self.missing_dependency_ids.is_empty(),
                    DependencyClosureStatusV1::BlockedCurrentness =>
                        self.missing_dependency_ids.is_empty() && has_stale,
                    DependencyClosureStatusV1::BlockedResourceLimit =>
                        self.missing_dependency_ids.is_empty() && !has_stale,
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

    pub(crate) fn expected_dependencies(&self) -> BTreeSet<SemanticDependencyReferenceV1> {
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
    /// Convert maps keyed by structured dependency references into canonical
    /// arrays before hashing. JSON object keys are strings; representing these
    /// bindings as explicit key/value records keeps the commitment format
    /// deterministic and executable instead of depending on serializer-specific
    /// treatment of non-string map keys.
    fn canonical_hash_material(&self) -> serde_json::Value {
        serde_json::json!({
            "schema_version": self.schema_version,
            "algorithm_version": self.algorithm_version,
            "closure_profile_commitment": self.closure_profile_commitment,
            "source_dkg_snapshot_commitment": self.source_dkg_snapshot_commitment,
            "projection_commitment": self.projection_commitment,
            "closure_identity_commitment": self.closure_identity_commitment,
            "semantic_environment_commitment": self.semantic_environment_commitment,
            "derivation_profile_commitment": self.derivation_profile_commitment,
            "root_node_ids": self.root_node_ids,
            "included_node_commitments": self.included_node_commitments,
            "included_node_ids": self.included_node_ids,
            "included_nodes": self.included_nodes,
            "included_d6p_receipt_commitments": self.included_d6p_receipt_commitments,
            "included_edge_commitments": self.included_edge_commitments,
            "included_edges": self.included_edges,
            "missing_dependency_ids": self.missing_dependency_ids,
            "missing_dependencies": self.missing_dependencies,
            "dependencies": self.dependencies,
            "dependency_resolutions": self.dependency_resolutions.iter()
                .map(|(dependency, resolution)| serde_json::json!({
                    "dependency": dependency,
                    "resolution": resolution,
                }))
                .collect::<Vec<_>>(),
            "resolution_evidence": self.resolution_evidence.iter()
                .map(|(dependency, evidence)| serde_json::json!({
                    "dependency": dependency,
                    "evidence": evidence,
                }))
                .collect::<Vec<_>>(),
            "status": self.status,
            "cycle_detected": self.cycle_detected,
            "claim_ceiling": self.claim_ceiling,
        })
    }

    pub fn closure_identity(&self) -> String {
        let identity = serde_json::json!({
            "schema_version": self.schema_version,
            "algorithm_version": self.algorithm_version,
            "canonicalization_version": D6X_CANONICALIZATION_VERSION,
            "closure_profile_commitment": self.closure_profile_commitment,
            "source_dkg_snapshot_commitment": self.source_dkg_snapshot_commitment,
            "semantic_environment_commitment": self.semantic_environment_commitment,
            "derivation_profile_commitment": self.derivation_profile_commitment,
            "root_node_ids": self.root_node_ids,
            "dependencies": self.dependencies,
            "missing_dependencies": self.missing_dependencies,
            "dependency_resolutions": self.dependency_resolutions.iter()
                .map(|(dependency, resolution)| serde_json::json!({
                    "dependency": dependency,
                    "resolution": resolution,
                }))
                .collect::<Vec<_>>(),
            "status": self.status,
            "cycle_detected": self.cycle_detected,
            "claim_ceiling": self.claim_ceiling,
        });
        canonical_sha256("d6x-closure-identity", &identity)
    }

    pub fn recompute(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.commitment.clear();
        canonical_sha256("d6x-dependency-closure", &unsigned.canonical_hash_material())
    }

    /// Verify this certificate against the exact source inputs that are claimed
    /// to produce it. valid() checks internal consistency; this stronger gate
    /// re-executes the D6X policy and compares the complete semantic result.
    ///
    /// Resolution evidence and the final certificate commitment are intentionally
    /// excluded from the semantic comparison: resolution evidence is audit-only
    /// metadata, and the commitment legitimately changes when that evidence is
    /// carried.
    pub fn verifies_against_sources(
        &self,
        projection: &QualifiedProjectionV1,
        environment: &SemanticEnvironmentV1,
        derivation_profile: &DerivationProfileV1,
        profile: &DependencyClosureProfileV1,
    ) -> bool {
        if !self.valid() {
            return false;
        }

        let Some(expected) =
            compute_dependency_closure(projection, environment, derivation_profile, profile)
        else {
            return false;
        };

        self.schema_version == expected.schema_version
            && self.algorithm_version == expected.algorithm_version
            && self.closure_profile_commitment == expected.closure_profile_commitment
            && self.source_dkg_snapshot_commitment == expected.source_dkg_snapshot_commitment
            && self.projection_commitment == expected.projection_commitment
            && self.closure_identity_commitment == expected.closure_identity_commitment
            && self.semantic_environment_commitment == expected.semantic_environment_commitment
            && self.derivation_profile_commitment == expected.derivation_profile_commitment
            && self.root_node_ids == expected.root_node_ids
            && self.included_node_commitments == expected.included_node_commitments
            && self.included_node_ids == expected.included_node_ids
            && self.included_nodes == expected.included_nodes
            && self.included_d6p_receipt_commitments == expected.included_d6p_receipt_commitments
            && self.included_edge_commitments == expected.included_edge_commitments
            && self.included_edges == expected.included_edges
            && self.missing_dependency_ids == expected.missing_dependency_ids
            && self.missing_dependencies == expected.missing_dependencies
            && self.dependencies == expected.dependencies
            && self.dependency_resolutions == expected.dependency_resolutions
            && self.status == expected.status
            && self.cycle_detected == expected.cycle_detected
            && self.claim_ceiling == expected.claim_ceiling
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

/// Qualified D6X entrypoint: before a closure can consume D6P receipt
/// commitments, require each selected receipt to be an exact projection of the
/// supplied committed D6P composition. This closes the finality->closure gap
/// where an opaque receipt commitment could otherwise be treated as sufficient
/// provenance.
///
/// This boundary does NOT independently reconstruct D6P from D6N/D6O source
/// state. The supplied composition is itself treated as the qualified D6P
/// source object; independent upstream authority reconstruction belongs at the
/// D6P admission boundary.
pub fn compute_dependency_closure_from_authoritative_d6p(
    projection: &QualifiedProjectionV1,
    environment: &SemanticEnvironmentV1,
    derivation_profile: &DerivationProfileV1,
    profile: &DependencyClosureProfileV1,
    d6p_receipts: &[CurrentFinalityEligibilityReceiptV1],
    d6p_compositions: &[FinalityEligibilityCompositionV1],
) -> Option<DependencyClosureCertificateV1> {
    compute_dependency_closure_from_authoritative_d6p_at_frontier(
        projection,
        environment,
        derivation_profile,
        profile,
        d6p_receipts,
        d6p_compositions,
        None,
    )
}

/// Strict qualified D6X entrypoint. When a current frontier is supplied,
/// every selected D6P receipt must prove that exact frontier. This prevents a
/// structurally valid historical receipt from being replayed as "current".
///
/// The caller remains responsible for establishing that the supplied D6P
/// composition was produced from authoritative D6N/D6O evidence. This function
/// verifies exact receipt-to-composition provenance and frontier binding, not
/// the upstream D6N/D6O authority claim.
pub fn compute_dependency_closure_from_authoritative_d6p_at_frontier(
    projection: &QualifiedProjectionV1,
    environment: &SemanticEnvironmentV1,
    derivation_profile: &DerivationProfileV1,
    profile: &DependencyClosureProfileV1,
    d6p_receipts: &[CurrentFinalityEligibilityReceiptV1],
    d6p_compositions: &[FinalityEligibilityCompositionV1],
    current_frontier_root: Option<&str>,
) -> Option<DependencyClosureCertificateV1> {
    if !projection.structurally_valid()
        || !environment.structurally_valid()
        || !derivation_profile.structurally_valid()
        || !profile.structurally_valid()
        || current_frontier_root.is_some_and(|root| root.trim().is_empty())
        || current_frontier_root.is_some_and(|root| environment.current_frontier_root.as_deref() != Some(root))
    {
        return None;
    }

    for commitment in &profile.required_d6p_receipt_commitments {
        if !projection.d6p_current_receipt_commitments.contains(commitment) {
            return None;
        }
        let matching_receipts = d6p_receipts
            .iter()
            .filter(|receipt| receipt.receipt_commitment == *commitment)
            .collect::<Vec<_>>();
        // Receipt identity is single-valued at the qualified boundary.
        // Multiple candidates for the same commitment are ambiguous input,
        // even when a first matching entry could otherwise be selected.
        let receipt = match matching_receipts.as_slice() {
            [receipt] => *receipt,
            [] => return None,
            _ => return None,
        };
        if let Some(expected_frontier) = current_frontier_root {
            if receipt.current_frontier_root != expected_frontier {
                return None;
            }
        }
        let matching_compositions = d6p_compositions
            .iter()
            .filter(|composition| composition.composition_commitment == receipt.composition_commitment)
            .collect::<Vec<_>>();
        // Composition identity is likewise single-valued: do not let input
        // order choose which provenance object supplies the receipt projection.
        let composition = match matching_compositions.as_slice() {
            [composition] => *composition,
            [] => return None,
            _ => return None,
        };
        if !verify_current_receipt_provenance_from_composition(receipt, composition) {
            return None;
        }
    }

    compute_dependency_closure(projection, environment, derivation_profile, profile)
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
        // CurrentOnly edges can mark a node Stale before that node reaches the
        // queue. Never allow dequeue-time bookkeeping to downgrade that stronger
        // resolution back to Present.
        dependency_resolutions
            .entry(node_dependency.clone())
            .and_modify(|resolution| {
                if *resolution != SemanticDependencyResolutionV1::Stale {
                    *resolution = SemanticDependencyResolutionV1::Present;
                }
            })
            .or_insert(SemanticDependencyResolutionV1::Present);

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
            // Overlapping rules resolve monotonically: CurrentOnly dominates Any.
            let Some(to) = projection.nodes.get(&edge.to_node_id) else {
                let dependency = SemanticDependencyReferenceV1::node(edge.to_node_id.clone(), None);
                missing_dependencies.insert(dependency.clone());
                dependency_resolutions.insert(dependency, SemanticDependencyResolutionV1::Missing);
                missing.insert(edge.to_node_id.clone());
                continue;
            };
            // A selected edge cannot outlive its target node. If the node
            // budget is already exhausted and this edge would introduce a new
            // target, stop before selecting or validating the edge. The
            // resource-bounded certificate then remains internally valid: every
            // selected edge has two selected endpoints.
            if !included_ids.contains(&edge.to_node_id)
                && included_ids.len() as u32 >= profile.max_nodes
            {
                resource_blocked = true;
                break;
            }
            if included_edges.len() as u32 >= profile.max_edges {
                resource_blocked = true;
                break;
            }
            // Only selected semantic edges are required to prove their own
            // commitment binding. Irrelevant/provenance candidates remain
            // outside the semantic closure boundary.
            if !edge.commitment_matches() {
                return None;
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
            // Node commitments deliberately remain compatible with the
            // upstream D6S model and therefore do not include the mutable
            // currentness annotations. CurrentOnly qualification must bind both
            // historical state and the node's declared frontier to the semantic
            // environment explicitly.
            let target_is_current = !to.historical_only
                && match (
                    environment.current_frontier_root.as_deref(),
                    to.current_frontier_root.as_deref(),
                ) {
                    (Some(expected), Some(actual)) => expected == actual,
                    // CurrentOnly cannot be established from omitted frontier
                    // metadata. Absence of either side is therefore fail-closed.
                    _ => false,
                };
            if requires_current && !target_is_current {
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
    // Constructors must not return a certificate that fails its own structural
    // invariant. In particular, selected node/edge commitment sets are
    // cardinality-checked against their identity maps, so duplicate selected
    // commitments are rejected at the constructor boundary rather than handed
    // downstream as an apparently qualified artifact.
    out.valid().then_some(out)
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

    fn valid_authoritative_d6p_fixture() -> (
        CurrentFinalityEligibilityReceiptV1,
        FinalityEligibilityCompositionV1,
    ) {
        let mut witness = crate::finality_eligibility_composition::FinalityWitnessEligibilityV1 {
            observation_id: "obs-1".into(),
            observer_id: "observer-1".into(),
            observer_generation_id: Some("gen-1".into()),
            d6n_observation_set_id: "set-1".into(),
            d6n_observation_set_commitment: "set-commit".into(),
            d6n_assessment_item_commitment: "assessment-item-1".into(),
            d6n_classification: crate::contestable_finality::ObservationClassificationV1::CorroboratingIndependent,
            d6o_eligibility_id: Some("elig-1".into()),
            d6o_disposition: Some(crate::observer_lifecycle::EvidenceEligibilityDispositionV1::EligibleCurrent),
            d6o_dependency_snapshot_id: Some("snapshot".into()),
            observation_frontier_root: "frontier".into(),
            current_frontier_root: "frontier".into(),
            lifecycle_profile_id: "lifecycle".into(),
            witness_commitment: String::new(),
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        witness.witness_commitment = witness.recomputed_commitment();

        let mut composition = FinalityEligibilityCompositionV1 {
            composition_id: "composition:set-1".into(),
            effect_id: "effect".into(),
            effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "gen-lifecycle".into(),
            route_id: "route".into(),
            provider_id: "provider".into(),
            provider_operation_id: "operation".into(),
            provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(),
            observation_set_id: "set-1".into(),
            observation_set_commitment: "set-commit".into(),
            d6n_assessment_commitment: "assessment".into(),
            lifecycle_profile_id: "lifecycle".into(),
            current_frontier_root: "frontier".into(),
            eligible_independent_count: 1,
            required_independent_observations: 1,
            preserved_contradictory_count: 0,
            witnesses: vec![witness],
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: Some("transition".into()),
            composition_commitment: String::new(),
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        composition.composition_commitment = composition.recomputed_commitment();

        let mut receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-1".into(),
            effect_id: "effect".into(),
            effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "gen-lifecycle".into(),
            route_id: "route".into(),
            provider_id: "provider".into(),
            provider_operation_id: "operation".into(),
            provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(),
            observation_set_id: "set-1".into(),
            observation_set_commitment: "set-commit".into(),
            d6n_assessment_commitment: "assessment".into(),
            composition_commitment: composition.composition_commitment.clone(),
            witness_eligibility_ids: ["elig-1".into()].into_iter().collect(),
            observer_generation_ids: ["gen-1".into()].into_iter().collect(),
            current_frontier_root: "frontier".into(),
            lifecycle_profile_id: "lifecycle".into(),
            eligible_independent_count: 1,
            preserved_contradictory_count: 0,
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition".into(),
            receipt_commitment: String::new(),
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        receipt.receipt_commitment = receipt.recomputed_commitment();

        (receipt, composition)
    }

    #[test]
    fn authoritative_d6p_accepts_bound_receipt_without_explicit_frontier() {
        let (mut projection, environment, derivation_profile) = projection(false);
        let receipt_commitment;
        let (receipt, composition) = valid_authoritative_d6p_fixture();
        receipt_commitment = receipt.receipt_commitment.clone();
        projection
            .d6p_current_receipt_commitments
            .insert(receipt_commitment.clone());

        let mut profile = profile(BTreeSet::new());
        profile
            .required_d6p_receipt_commitments
            .insert(receipt_commitment);

        let closure = compute_dependency_closure_from_authoritative_d6p(
            &projection,
            &environment,
            &derivation_profile,
            &profile,
            std::slice::from_ref(&receipt),
            std::slice::from_ref(&composition),
        )
        .expect("non-strict authoritative D6P path must accept a bound receipt");

        assert_eq!(closure.status, DependencyClosureStatusV1::Complete);
        assert_eq!(
            closure.included_d6p_receipt_commitments,
            [receipt.receipt_commitment.clone()].into_iter().collect()
        );
    }

    #[test]
    fn authoritative_d6p_rejects_frontier_mismatch_with_environment() {
        let (mut p, mut e, d) = projection(false);
        let receipt_commitment = "receipt-1".to_string();
        p.d6p_current_receipt_commitments.insert(receipt_commitment.clone());

        e.current_frontier_root = Some("frontier-current".into());

        let mut profile = profile(BTreeSet::new());
        profile.required_d6p_receipt_commitments.insert(receipt_commitment.clone());

        let receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-id".into(),
            effect_id: "effect".into(),
            effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "generation".into(),
            route_id: "route".into(),
            provider_id: "provider".into(),
            provider_operation_id: "operation".into(),
            provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(),
            observation_set_id: "set".into(),
            observation_set_commitment: "set-commitment".into(),
            d6n_assessment_commitment: "assessment".into(),
            composition_commitment: "composition".into(),
            witness_eligibility_ids: ["eligibility".into()].into_iter().collect(),
            observer_generation_ids: ["generation".into()].into_iter().collect(),
            current_frontier_root: "frontier-current".into(),
            lifecycle_profile_id: "lifecycle".into(),
            eligible_independent_count: 1,
            preserved_contradictory_count: 0,
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition".into(),
            receipt_commitment,
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };

        assert!(
            compute_dependency_closure_from_authoritative_d6p_at_frontier(
                &p,
                &e,
                &d,
                &profile,
                std::slice::from_ref(&receipt),
                &[],
                Some("frontier-other"),
            )
            .is_none()
        );
    }

    #[test]
    fn authoritative_d6p_rejects_duplicate_receipt_identity() {
        let (mut p, mut e, d) = projection(false);
        let receipt_commitment = "receipt-duplicate".to_string();
        p.d6p_current_receipt_commitments.insert(receipt_commitment.clone());
        e.current_frontier_root = Some("frontier-current".into());

        let mut profile = profile(BTreeSet::new());
        profile.required_d6p_receipt_commitments.insert(receipt_commitment.clone());

        let receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-id".into(),
            effect_id: "effect".into(),
            effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "generation".into(),
            route_id: "route".into(),
            provider_id: "provider".into(),
            provider_operation_id: "operation".into(),
            provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(),
            observation_set_id: "set".into(),
            observation_set_commitment: "set-commitment".into(),
            d6n_assessment_commitment: "assessment".into(),
            composition_commitment: "composition".into(),
            witness_eligibility_ids: ["eligibility".into()].into_iter().collect(),
            observer_generation_ids: ["generation".into()].into_iter().collect(),
            current_frontier_root: "frontier-current".into(),
            lifecycle_profile_id: "lifecycle".into(),
            eligible_independent_count: 1,
            preserved_contradictory_count: 0,
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition".into(),
            receipt_commitment,
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };

        assert!(
            compute_dependency_closure_from_authoritative_d6p_at_frontier(
                &p,
                &e,
                &d,
                &profile,
                &[receipt.clone(), receipt],
                &[],
                Some("frontier-current"),
            )
            .is_none(),
            "qualified D6P admission must reject duplicate receipt identities"
        );
    }

    #[test]
    fn authoritative_d6p_rejects_duplicate_composition_identity() {
        let (mut p, e, d) = projection(false);
        let receipt_commitment = "receipt-composition-duplicate".to_string();
        p.d6p_current_receipt_commitments.insert(receipt_commitment.clone());

        let mut profile = profile(BTreeSet::new());
        profile.required_d6p_receipt_commitments.insert(receipt_commitment.clone());

        let receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-id".into(),
            effect_id: "effect".into(),
            effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "generation".into(),
            route_id: "route".into(),
            provider_id: "provider".into(),
            provider_operation_id: "operation".into(),
            provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(),
            observation_set_id: "set".into(),
            observation_set_commitment: "set-commitment".into(),
            d6n_assessment_commitment: "assessment".into(),
            composition_commitment: "composition-duplicate".into(),
            witness_eligibility_ids: ["eligibility".into()].into_iter().collect(),
            observer_generation_ids: ["generation".into()].into_iter().collect(),
            current_frontier_root: "frontier".into(),
            lifecycle_profile_id: "lifecycle".into(),
            eligible_independent_count: 1,
            preserved_contradictory_count: 0,
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition".into(),
            receipt_commitment,
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };

        let composition_a = FinalityEligibilityCompositionV1 {
            composition_id: "composition:set".into(),
            effect_id: "effect".into(),
            effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "generation".into(),
            route_id: "route".into(),
            provider_id: "provider".into(),
            provider_operation_id: "operation".into(),
            provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(),
            observation_set_id: "set".into(),
            observation_set_commitment: "set-commitment".into(),
            d6n_assessment_commitment: "assessment".into(),
            lifecycle_profile_id: "lifecycle".into(),
            current_frontier_root: "frontier".into(),
            eligible_independent_count: 1,
            required_independent_observations: 1,
            preserved_contradictory_count: 0,
            witnesses: Vec::new(),
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: Some("transition".into()),
            composition_commitment: "composition-duplicate".into(),
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        let mut composition_b = composition_a.clone();
        composition_b.effect_id = "other-effect".into();

        assert!(
            compute_dependency_closure_from_authoritative_d6p_at_frontier(
                &p,
                &e,
                &d,
                &profile,
                std::slice::from_ref(&receipt),
                &[composition_a, composition_b],
                Some("frontier"),
            )
            .is_none(),
            "qualified D6P admission must reject ambiguous composition identity",
        );
    }

    #[test]
    fn authoritative_d6p_rejects_replayed_historical_frontier() {
        let (mut p, e, d) = projection(false);
        let receipt_commitment = "receipt-1".to_string();
        p.d6p_current_receipt_commitments.insert(receipt_commitment.clone());

        let mut profile = profile(BTreeSet::new());
        profile.required_d6p_receipt_commitments.insert(receipt_commitment.clone());

        let receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-id".into(),
            effect_id: "effect".into(),
            effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "generation".into(),
            route_id: "route".into(),
            provider_id: "provider".into(),
            provider_operation_id: "operation".into(),
            provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(),
            observation_set_id: "set".into(),
            observation_set_commitment: "set-commitment".into(),
            d6n_assessment_commitment: "assessment".into(),
            composition_commitment: "composition".into(),
            witness_eligibility_ids: ["eligibility".into()].into_iter().collect(),
            observer_generation_ids: ["generation".into()].into_iter().collect(),
            current_frontier_root: "frontier-old".into(),
            lifecycle_profile_id: "lifecycle".into(),
            eligible_independent_count: 1,
            preserved_contradictory_count: 0,
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition".into(),
            receipt_commitment,
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };

        assert!(
            compute_dependency_closure_from_authoritative_d6p_at_frontier(
                &p,
                &e,
                &d,
                &profile,
                std::slice::from_ref(&receipt),
                &[],
                Some("frontier-current"),
            )
            .is_none()
        );
    }

    #[test]
    fn source_verifier_rejects_self_consistent_root_substitution() {
        let (a, e, d) = projection(false);
        let p = profile(BTreeSet::new());
        let baseline = compute_dependency_closure(&a, &e, &d, &p).unwrap();

        let mut forged = baseline.clone();
        forged.root_node_ids = BTreeSet::from(["dep".to_string()]);
        forged.closure_identity_commitment = forged.closure_identity();
        forged.commitment = forged.recompute();

        assert!(forged.valid());
        assert!(!forged.verifies_against_sources(&a, &e, &d, &p));
    }

    #[test]
    fn source_verifier_rejects_self_consistent_status_substitution() {
        let (a, e, d) = projection(false);
        let p = profile(BTreeSet::new());
        let baseline = compute_dependency_closure(&a, &e, &d, &p).unwrap();

        let mut forged = baseline.clone();
        forged.status = DependencyClosureStatusV1::BlockedResourceLimit;
        forged.closure_identity_commitment = forged.closure_identity();
        forged.commitment = forged.recompute();

        assert!(forged.valid());
        assert!(!forged.verifies_against_sources(&a, &e, &d, &p));
    }

    #[test]
    fn source_verifier_rejects_self_consistent_cycle_claim() {
        let (a, e, d) = projection(false);
        let p = profile(BTreeSet::new());
        let baseline = compute_dependency_closure(&a, &e, &d, &p).unwrap();

        let mut forged = baseline.clone();
        forged.cycle_detected = true;
        forged.closure_identity_commitment = forged.closure_identity();
        forged.commitment = forged.recompute();

        assert!(forged.valid());
        assert!(!forged.verifies_against_sources(&a, &e, &d, &p));
    }

    #[test]
    fn source_verifier_allows_audit_only_resolution_evidence() {
        let (a, e, d) = projection(false);
        let p = profile(BTreeSet::new());
        let baseline = compute_dependency_closure(&a, &e, &d, &p).unwrap();
        let mut evidenced = baseline.clone();
        let dependency = SemanticDependencyReferenceV1::node(
            "root",
            Some(
                a.nodes
                    .get("root")
                    .expect("root")
                    .node_commitment
                    .clone(),
            ),
        );
        evidenced.resolution_evidence.insert(
            dependency,
            SemanticDependencyResolutionEvidenceV1 {
                retrieval_reference: Some("runtime://resolver/source-check".into()),
                observed_commitment: Some(
                    a.nodes
                        .get("root")
                        .expect("root")
                        .node_commitment
                        .clone(),
                ),
                qualification_context_commitment: Some("qualification-source-check".into()),
            },
        );
        evidenced.commitment = evidenced.recompute();

        assert!(evidenced.valid());
        assert!(evidenced.verifies_against_sources(&a, &e, &d, &p));
    }

    #[test]
    fn resolution_evidence_with_wrong_observed_commitment_fails_closed() {
        let (projection, environment, derivation_profile) = projection(false);
        let profile = profile(BTreeSet::new());
        let mut certificate =
            compute_dependency_closure(&projection, &environment, &derivation_profile, &profile)
                .expect("baseline closure");

        let dependency = SemanticDependencyReferenceV1::node(
            "root",
            Some(
                projection
                    .nodes
                    .get("root")
                    .expect("root")
                    .node_commitment
                    .clone(),
            ),
        );
        certificate.resolution_evidence.insert(
            dependency,
            SemanticDependencyResolutionEvidenceV1 {
                retrieval_reference: Some("runtime://resolver/wrong-observed-commitment".into()),
                observed_commitment: Some("commitment-from-another-object".into()),
                qualification_context_commitment: Some("qualification-context-1".into()),
            },
        );
        certificate.commitment = certificate.recompute();

        assert!(!certificate.valid());
        assert!(!certificate.verifies_against_sources(
            &projection,
            &environment,
            &derivation_profile,
            &profile,
        ));
    }

    #[test]
    fn structured_dependency_maps_are_hashable_and_deterministic() {
        let (projection, environment, derivation_profile) = projection(false);
        let profile = profile(BTreeSet::new());
        let closure = compute_dependency_closure(
            &projection,
            &environment,
            &derivation_profile,
            &profile,
        )
        .expect("baseline closure");

        let first = closure.recompute();
        assert_eq!(first, closure.recompute());
        assert!(closure.commitment == first);

        let mut with_evidence = closure.clone();
        let dependency = SemanticDependencyReferenceV1::node(
            "root",
            Some(
                projection
                    .nodes
                    .get("root")
                    .expect("root")
                    .node_commitment
                    .clone(),
            ),
        );
        with_evidence.resolution_evidence.insert(
            dependency,
            SemanticDependencyResolutionEvidenceV1 {
                retrieval_reference: Some("runtime://resolver/golden".into()),
                observed_commitment: None,
                qualification_context_commitment: Some("qualification-1".into()),
            },
        );
        assert_eq!(
            closure.closure_identity_commitment,
            with_evidence.closure_identity(),
            "audit-only retrieval evidence must not alter semantic closure identity",
        );
        assert_ne!(first, with_evidence.recompute());
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
    fn one_over_node_resource_limit_blocks_after_required_work_remains() {
        let (a, e, d) = projection(false);
        let mut p = profile(BTreeSet::new());
        p.max_nodes = 1;
        p.max_edges = 1;

        let c = compute_dependency_closure(&a, &e, &d, &p).unwrap();

        assert_eq!(c.status, DependencyClosureStatusV1::BlockedResourceLimit);
        assert_eq!(c.included_node_ids, BTreeSet::from(["root".to_string()]));
        assert!(c.included_edges.is_empty());
        assert!(c.valid());
    }

    #[test]
    fn node_budget_does_not_select_edge_to_unincluded_target() {
        let (a, e, d) = projection(false);
        let mut p = profile(BTreeSet::new());
        p.max_nodes = 1;
        p.max_edges = 1;

        let c = compute_dependency_closure(&a, &e, &d, &p).unwrap();

        assert_eq!(c.status, DependencyClosureStatusV1::BlockedResourceLimit);
        assert_eq!(c.included_node_ids, BTreeSet::from(["root".to_string()]));
        assert!(c.included_edges.is_empty());
        assert!(c.valid());
        assert!(c.dependencies.iter().all(|dependency| {
            dependency.kind != SemanticDependencyKindV1::Edge
        }));
    }

    #[test]
    fn one_over_edge_resource_limit_blocks_only_after_an_additional_selected_edge_exists() {
        let (mut a, e, d) = projection(false);
        a.nodes.insert("dep-2".into(), QualifiedNodeV1 {
            node_id: "dep-2".into(),
            kind: ClaimGraphNodeKindV1::Evidence,
            content_commitment: "content-dep-2".into(),
            node_commitment: "commit-dep-2".into(),
            historical_only: false,
            current_frontier_root: Some("frontier".into()),
            claim_ceiling: D6X_CLAIM_CEILING.into(),
        });
        a.edges.insert("e2".into(), QualifiedEdgeV1 {
            edge_id: "e2".into(),
            from_node_id: "root".into(),
            to_node_id: "dep-2".into(),
            kind: ClaimGraphEdgeKindV1::Supports,
            edge_commitment: "edge-e2".into(),
            claim_ceiling: D6X_CLAIM_CEILING.into(),
        });

        let mut p = profile(BTreeSet::new());
        p.rules = [
            DependencyRuleV1 {
                edge_kind: ClaimGraphEdgeKindV1::Supports,
                from_kind: Some(ClaimGraphNodeKindV1::Statement),
                to_kind: Some(ClaimGraphNodeKindV1::Evidence),
                currentness: DependencyCurrentnessV1::Any,
            },
        ].into_iter().collect();
        p.max_nodes = 8;
        p.max_edges = 1;

        let c = compute_dependency_closure(&a, &e, &d, &p).unwrap();

        assert_eq!(c.status, DependencyClosureStatusV1::BlockedResourceLimit);
        assert_eq!(c.included_edges.len(), 1);
        assert!(c.valid());
    }

    #[test]
    fn duplicate_selected_node_commitment_fails_closed_at_constructor() {
        let (mut a, e, d) = projection(false);
        a.nodes.get_mut("dep").unwrap().node_commitment = "commit-root".into();

        let p = profile(BTreeSet::new());
        assert!(compute_dependency_closure(&a, &e, &d, &p).is_none());
    }

    #[test]
    fn duplicate_selected_edge_commitment_fails_closed_at_constructor() {
        let (mut a, e, d) = projection(false);
        a.edges.insert("e2".into(), QualifiedEdgeV1 {
            edge_id: "e2".into(),
            from_node_id: "root".into(),
            to_node_id: "dep".into(),
            kind: ClaimGraphEdgeKindV1::Supports,
            edge_commitment: "edge-e1".into(),
            claim_ceiling: D6X_CLAIM_CEILING.into(),
        });

        let p = profile(BTreeSet::new());
        assert!(compute_dependency_closure(&a, &e, &d, &p).is_none());
    }

    #[test]
    fn blocked_currentness_cannot_hide_missing_dependency_precedence() {
        let (a, e, d) = projection(false);
        let mut c = compute_dependency_closure(
            &a,
            &e,
            &d,
            &profile(["missing".into()].into_iter().collect()),
        )
        .unwrap();

        c.status = DependencyClosureStatusV1::BlockedCurrentness;
        c.closure_identity_commitment = c.closure_identity();
        c.commitment = c.recompute();

        assert!(!c.valid());
    }

    #[test]
    fn blocked_resource_limit_cannot_hide_missing_or_stale_precedence() {
        let (a, e, d) = projection(false);
        let mut c = compute_dependency_closure(
            &a,
            &e,
            &d,
            &profile(["missing".into()].into_iter().collect()),
        )
        .unwrap();

        c.status = DependencyClosureStatusV1::BlockedResourceLimit;
        c.closure_identity_commitment = c.closure_identity();
        c.commitment = c.recompute();

        assert!(!c.valid());

        let (mut a, e, d) = projection(false);
        a.nodes.get_mut("dep").unwrap().historical_only = true;
        let mut current_profile = profile(BTreeSet::new());
        current_profile.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::CurrentOnly,
        }]
        .into_iter()
        .collect();

        let mut c = compute_dependency_closure(&a, &e, &d, &current_profile).unwrap();
        c.status = DependencyClosureStatusV1::BlockedResourceLimit;
        c.closure_identity_commitment = c.closure_identity();
        c.commitment = c.recompute();

        assert!(!c.valid());
    }

    #[derive(Debug, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct GoldenCorpusV1 {
        schema_version: String,
        d6x_schema_version: String,
        d6x_algorithm_version: String,
        canonicalization_version: String,
        hash_domain_prefix_hex: String,
        closure_identity_domain: String,
        certificate_domain: String,
        claim_ceiling: String,
        generation_note: String,
        vectors: Vec<GoldenVectorV1>,
        fixtures: GoldenFixturesV1,
        recipe_contract: GoldenRecipeContractV1,
    }

    #[derive(Debug, Clone, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct GoldenVectorV1 {
        id: String,
        description: String,
        profile_variant: String,
        projection_variant: String,
        expected: GoldenExpectedV1,
        recipe: GoldenRecipeV1,
    }

    #[derive(Debug, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct GoldenRecipeContractV1 {
        baseline_projection: String,
        mutation_semantics: String,
        cross_runtime_rule: String,
        semantic_derivation: String,
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct GoldenFixturesV1 {
        baseline_projection: QualifiedProjectionV1,
        semantic_environment: SemanticEnvironmentV1,
        derivation_profile: DerivationProfileV1,
    }

    #[derive(Debug, Clone, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct GoldenRecipeV1 {
        profile_id: String,
        profile_version: String,
        excluded_boundary_policy: String,
        claim_ceiling: String,
        root_node_ids: Vec<String>,
        required_node_ids: Vec<String>,
        #[serde(default)]
        required_d6p_receipt_commitments: Vec<String>,
        #[serde(default)]
        projection_d6p_receipt_commitments: Vec<String>,
        rule: GoldenRuleV1,
        max_nodes: u32,
        max_edges: u32,
        projection_mutations: Vec<GoldenProjectionMutationV1>,
    }

    #[derive(Debug, Clone, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct GoldenRuleV1 {
        edge_kind: ClaimGraphEdgeKindV1,
        from_kind: Option<ClaimGraphNodeKindV1>,
        to_kind: Option<ClaimGraphNodeKindV1>,
        currentness: DependencyCurrentnessV1,
    }

    #[derive(Debug, Clone, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct GoldenProjectionMutationV1 {
        op: String,
        edge_id: Option<String>,
        from_node_id: Option<String>,
        to_node_id: Option<String>,
        kind: Option<ClaimGraphEdgeKindV1>,
        edge_commitment: Option<String>,
        node_id: Option<String>,
        fields: Option<BTreeMap<String, serde_json::Value>>,
    }

    #[derive(Debug, Clone, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct GoldenExpectedV1 {
        status: DependencyClosureStatusV1,
        included_node_ids: Vec<String>,
        included_edge_ids: Vec<String>,
        missing_dependency_ids: Vec<String>,
        cycle_detected: bool,
        closure_identity_sha256: String,
        certificate_sha256: String,
    }

    #[test]
    fn golden_vector_corpus_is_well_formed_and_covers_acceptance_cases() {
        let corpus_text = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/d6x_qualified_closure_golden_vectors.json"
        ));
        let corpus: GoldenCorpusV1 =
            serde_json::from_str(corpus_text).expect("D6X golden vector corpus must parse");

        // Re-compare the typed fixture subtree against its serialized form so
        // Serde cannot silently discard an unknown field from the corpus.
        let raw_corpus: serde_json::Value =
            serde_json::from_str(corpus_text).expect("D6X golden corpus JSON must remain valid");
        assert_eq!(
            raw_corpus.get("fixtures"),
            Some(
                &serde_json::to_value(&corpus.fixtures)
                    .expect("typed D6X fixtures must serialize")
            ),
            "D6X golden fixtures must contain exactly the declared typed schema"
        );

        assert_eq!(corpus.schema_version, "D6X-GOLDEN-1");
        assert_eq!(corpus.d6x_schema_version, D6X_SCHEMA_VERSION);
        assert_eq!(corpus.d6x_algorithm_version, D6X_ALGORITHM_VERSION);
        assert_eq!(corpus.canonicalization_version, D6X_CANONICALIZATION_VERSION);
        assert_eq!(
            corpus.hash_domain_prefix_hex,
            "4d5943454c49582d494e54454752414c2d4436532d524543454950542d563100"
        );
        assert_eq!(corpus.closure_identity_domain, "d6x-closure-identity");
        assert_eq!(corpus.certificate_domain, "d6x-dependency-closure");
        assert_eq!(corpus.claim_ceiling, D6X_CLAIM_CEILING);
        assert!(!corpus.generation_note.is_empty());
        assert!(!corpus.fixtures.baseline_projection.projection_id.is_empty());
        assert!(!corpus.fixtures.semantic_environment.semantic_profile_id.is_empty());
        assert!(!corpus.fixtures.derivation_profile.profile_id.is_empty());
        assert!(!corpus.recipe_contract.baseline_projection.is_empty());
        assert!(!corpus.recipe_contract.mutation_semantics.is_empty());
        assert!(!corpus.recipe_contract.cross_runtime_rule.is_empty());
        assert!(!corpus.recipe_contract.semantic_derivation.is_empty());

        let ids = corpus
            .vectors
            .iter()
            .map(|vector| vector.id.as_str())
            .collect::<BTreeSet<_>>();
        assert_eq!(
            ids.len(),
            corpus.vectors.len(),
            "D6X golden vector IDs must be unique; duplicate IDs would be order-dependent"
        );

        for vector in &corpus.vectors {
            assert!(!vector.id.is_empty());
            assert!(!vector.description.is_empty());
            assert!(!vector.profile_variant.is_empty());
            assert!(!vector.projection_variant.is_empty());
            assert!(!vector.recipe.profile_id.trim().is_empty());
            assert!(!vector.recipe.profile_version.trim().is_empty());
            assert!(!vector.recipe.excluded_boundary_policy.trim().is_empty());
            assert_eq!(vector.recipe.claim_ceiling, D6X_CLAIM_CEILING);
            assert!(!vector.recipe.root_node_ids.is_empty());
            assert!(vector.recipe.root_node_ids.iter().all(|id| !id.is_empty()));
            assert!(vector.recipe.required_node_ids.iter().all(|id| !id.is_empty()));
            assert!(vector.recipe.required_d6p_receipt_commitments.iter().all(|id| !id.is_empty()));
            assert_eq!(
                vector.recipe.required_d6p_receipt_commitments.iter().collect::<BTreeSet<_>>().len(),
                vector.recipe.required_d6p_receipt_commitments.len(),
                "duplicate required D6P receipt commitments are ambiguous: {}",
                vector.id
            );
            assert!(vector.recipe.projection_d6p_receipt_commitments.iter().all(|id| !id.is_empty()));
            assert_eq!(
                vector.recipe.projection_d6p_receipt_commitments.iter().collect::<BTreeSet<_>>().len(),
                vector.recipe.projection_d6p_receipt_commitments.len(),
                "duplicate projected D6P receipt commitments are ambiguous: {}",
                vector.id
            );
            assert!(vector.recipe.max_nodes > 0);
            assert!(vector.recipe.max_edges > 0);
            let declared_rule = DependencyRuleV1 {
                edge_kind: vector.recipe.rule.edge_kind,
                from_kind: vector.recipe.rule.from_kind,
                to_kind: vector.recipe.rule.to_kind,
                currentness: vector.recipe.rule.currentness,
            };
            assert!(
                declared_rule.structurally_valid(),
                "golden recipe rule must satisfy D6X rule shape: {}",
                vector.id
            );
            let mut mutation_edge_ids = BTreeSet::new();
            let mut mutation_field_targets = BTreeSet::new();
            for mutation in &vector.recipe.projection_mutations {
                match mutation.op.as_str() {
                    "add_edge" => {
                        let edge_id = mutation.edge_id.as_deref().expect("validated add_edge edge_id");
                        assert!(
                            mutation_edge_ids.insert(edge_id),
                            "golden recipe must not contain duplicate add_edge IDs: {}",
                            vector.id
                        );
                        assert!(mutation.edge_id.as_deref().is_some_and(non_empty));
                        assert!(mutation.from_node_id.as_deref().is_some_and(non_empty));
                        assert!(mutation.to_node_id.as_deref().is_some_and(non_empty));
                        assert!(mutation.kind.is_some());
                        assert!(mutation.edge_commitment.as_deref().is_some_and(non_empty));
                        assert!(mutation.node_id.is_none());
                        assert!(mutation.fields.is_none());
                    }
                    "set_node" => {
                        assert!(mutation.edge_id.is_none());
                        assert!(mutation.from_node_id.is_none());
                        assert!(mutation.to_node_id.is_none());
                        assert!(mutation.kind.is_none());
                        assert!(mutation.edge_commitment.is_none());
                        let node_id = mutation
                            .node_id
                            .as_deref()
                            .expect("validated set_node node_id");
                        assert!(non_empty(node_id));
                        let fields = mutation
                            .fields
                            .as_ref()
                            .filter(|fields| !fields.is_empty())
                            .unwrap_or_else(|| panic!("set_node mutation must declare fields: {}", vector.id));
                        for (field, value) in fields {
                            assert!(
                                mutation_field_targets.insert((node_id, field.as_str())),
                                "golden recipe must not assign the same node field twice: {}:{field}",
                                vector.id
                            );
                            match field.as_str() {
                                "historical_only" => {
                                    assert!(value.is_boolean(), "historical_only must be boolean: {}", vector.id);
                                }
                                "current_frontier_root" => {
                                    assert!(
                                        value.is_null() || value.as_str().is_some_and(non_empty),
                                        "current_frontier_root must be null or non-empty string: {}",
                                        vector.id
                                    );
                                }
                                _ => panic!("unsupported set_node field in golden recipe {}: {field}", vector.id),
                            }
                        }
                    }
                    op => panic!("unsupported D6X golden recipe mutation: {op}"),
                }
            }
            assert!(!vector.expected.included_node_ids.iter().any(String::is_empty));
            assert!(!vector.expected.included_edge_ids.iter().any(String::is_empty));
            assert!(!vector.expected.missing_dependency_ids.iter().any(String::is_empty));
            assert!(is_canonical_sha256_commitment(
                &vector.expected.closure_identity_sha256
            ));
            assert!(is_canonical_sha256_commitment(
                &vector.expected.certificate_sha256
            ));
        }

        for required in [
            "baseline-complete",
            "edge-free-complete",
            "missing-required-node",
            "cyclic-complete",
            "currentness-historical",
            "currentness-frontier-mismatch",
            "currentness-frontier-omitted",
            "resource-node-limit",
            "resource-edge-limit",
        ] {
            assert!(ids.contains(required), "missing D6X golden vector: {required}");
        }
    }

    fn assert_golden_vector(
        corpus: &GoldenCorpusV1,
        id: &str,
        closure: &DependencyClosureCertificateV1,
    ) {
        let vector = corpus
            .vectors
            .iter()
            .find(|vector| vector.id == id)
            .unwrap_or_else(|| panic!("missing D6X golden vector: {id}"));

        assert_eq!(closure.status, vector.expected.status, "status: {id}");
        assert_eq!(
            closure.included_node_ids.iter().cloned().collect::<Vec<_>>(),
            vector.expected.included_node_ids,
            "included nodes: {id}"
        );
        assert_eq!(
            closure.included_edges.keys().cloned().collect::<Vec<_>>(),
            vector.expected.included_edge_ids,
            "included edges: {id}"
        );
        assert_eq!(
            closure.missing_dependency_ids.iter().cloned().collect::<Vec<_>>(),
            vector.expected.missing_dependency_ids,
            "missing dependencies: {id}"
        );
        assert_eq!(closure.cycle_detected, vector.expected.cycle_detected, "cycle: {id}");
        assert_eq!(
            closure.closure_identity_commitment,
            vector.expected.closure_identity_sha256,
            "closure identity: {id}"
        );
        assert_eq!(
            closure.commitment,
            vector.expected.certificate_sha256,
            "certificate commitment: {id}"
        );
    }

    fn execute_golden_recipe(
        corpus: &GoldenCorpusV1,
        vector: &GoldenVectorV1,
    ) -> (
        QualifiedProjectionV1,
        SemanticEnvironmentV1,
        DerivationProfileV1,
        DependencyClosureProfileV1,
    ) {
        let environment = corpus.fixtures.semantic_environment.clone();
        let derivation_profile = corpus.fixtures.derivation_profile.clone();

        let mut projection = corpus.fixtures.baseline_projection.clone();

        assert_eq!(
            projection.semantic_environment_commitment,
            environment.commitment(),
            "golden fixture must bind the exact D6S semantic-environment commitment"
        );
        assert_eq!(
            projection.derivation_profile_commitment,
            derivation_profile.commitment(),
            "golden fixture must bind the exact D6S derivation-profile commitment"
        );

        let mut projection_d6p_receipt_ids = BTreeSet::new();
        for receipt in &vector.recipe.projection_d6p_receipt_commitments {
            assert!(
                projection_d6p_receipt_ids.insert(receipt.as_str()),
                "duplicate projected D6P receipt commitment: {receipt}"
            );
            assert!(
                projection.d6p_current_receipt_commitments.insert(receipt.clone()),
                "golden recipe may not add the same D6P receipt commitment twice: {receipt}"
            );
        }

        let mut projection_value = serde_json::to_value(&projection)
            .expect("typed golden baseline projection must serialize");
        let projection_object = projection_value
            .as_object_mut()
            .expect("typed golden baseline projection must serialize as an object");

        for mutation in &vector.recipe.projection_mutations {
            match mutation.op.as_str() {
                "add_edge" => {
                    let edge_id = mutation.edge_id.clone().expect("validated add_edge edge_id");
                    let from_node_id = mutation
                        .from_node_id
                        .clone()
                        .expect("validated add_edge from_node_id");
                    let to_node_id = mutation
                        .to_node_id
                        .clone()
                        .expect("validated add_edge to_node_id");
                    let kind = mutation.kind.expect("validated add_edge kind");
                    let edge_commitment = mutation
                        .edge_commitment
                        .clone()
                        .expect("validated add_edge edge_commitment");
                    let nodes = projection_object
                        .get("nodes")
                        .and_then(serde_json::Value::as_object)
                        .expect("golden projection nodes must be an object");
                    assert!(
                        nodes.contains_key(&from_node_id),
                        "golden add_edge source node is missing: {from_node_id}"
                    );
                    assert!(
                        nodes.contains_key(&to_node_id),
                        "golden add_edge target node is missing: {to_node_id}"
                    );
                    let edges = projection_object
                        .get_mut("edges")
                        .and_then(serde_json::Value::as_object_mut)
                        .expect("golden projection edges must be an object");
                    assert!(
                        edges.get(&edge_id).is_none(),
                        "golden add_edge must not overwrite an existing edge: {edge_id}"
                    );
                    edges.insert(
                        edge_id.clone(),
                        serde_json::json!({
                            "edge_id": edge_id,
                            "from_node_id": from_node_id,
                            "to_node_id": to_node_id,
                            "kind": kind,
                            "edge_commitment": edge_commitment,
                            "claim_ceiling": D6X_CLAIM_CEILING,
                        }),
                    );
                }
                "set_node" => {
                    let node_id = mutation.node_id.as_ref().expect("validated set_node node_id");
                    let fields = mutation.fields.as_ref().expect("validated set_node fields");
                    let nodes = projection_object
                        .get_mut("nodes")
                        .and_then(serde_json::Value::as_object_mut)
                        .expect("golden projection nodes must be an object");
                    let node = nodes
                        .get_mut(node_id)
                        .and_then(serde_json::Value::as_object_mut)
                        .unwrap_or_else(|| panic!("golden set_node target is missing: {node_id}"));
                    for (field, value) in fields {
                        node.insert(field.clone(), value.clone());
                    }
                }
                op => panic!("validated corpus contained unsupported mutation: {op}"),
            }
        }

        let projection: QualifiedProjectionV1 = serde_json::from_value(projection_value)
            .expect("golden projection recipe must deserialize");

        let rule = DependencyRuleV1 {
            edge_kind: vector.recipe.rule.edge_kind,
            from_kind: vector.recipe.rule.from_kind,
            to_kind: vector.recipe.rule.to_kind,
            currentness: vector.recipe.rule.currentness,
        };
        let profile = DependencyClosureProfileV1 {
            profile_id: vector.recipe.profile_id.clone(),
            version: vector.recipe.profile_version.clone(),
            root_node_ids: vector.recipe.root_node_ids.iter().cloned().collect(),
            required_node_ids: vector.recipe.required_node_ids.iter().cloned().collect(),
            required_d6p_receipt_commitments: vector
                .recipe
                .required_d6p_receipt_commitments
                .iter()
                .cloned()
                .collect(),
            rules: [rule].into_iter().collect(),
            excluded_boundary_policy: vector.recipe.excluded_boundary_policy.clone(),
            max_nodes: vector.recipe.max_nodes,
            max_edges: vector.recipe.max_edges,
            claim_ceiling: vector.recipe.claim_ceiling.clone(),
        };

        (projection, environment, derivation_profile, profile)
    }

    #[test]
    fn golden_recipe_d6p_requirement_is_executable() {
        let corpus: GoldenCorpusV1 = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/d6x_qualified_closure_golden_vectors.json"
        )))
        .expect("D6X golden vector corpus must parse");

        let mut missing = corpus
            .vectors
            .iter()
            .find(|vector| vector.id == "baseline-complete")
            .expect("baseline golden vector")
            .clone();
        missing.recipe.required_d6p_receipt_commitments = vec!["receipt-required".into()];

        let (projection, environment, derivation_profile, profile) =
            execute_golden_recipe(&corpus, &missing);
        let blocked = compute_dependency_closure(
            &projection,
            &environment,
            &derivation_profile,
            &profile,
        )
        .expect("missing D6P receipt should still yield a blocked closure certificate");
        assert_eq!(blocked.status, DependencyClosureStatusV1::BlockedMissingDependency);
        let receipt = SemanticDependencyReferenceV1::d6p_receipt("receipt-required");
        assert_eq!(
            blocked.dependency_resolutions.get(&receipt),
            Some(&SemanticDependencyResolutionV1::Missing)
        );

        missing.recipe.projection_d6p_receipt_commitments = vec!["receipt-required".into()];
        let (projection, environment, derivation_profile, profile) =
            execute_golden_recipe(&corpus, &missing);
        let complete = compute_dependency_closure(
            &projection,
            &environment,
            &derivation_profile,
            &profile,
        )
        .expect("present D6P receipt should yield a complete closure");
        assert_eq!(complete.status, DependencyClosureStatusV1::Complete);
        assert!(complete.included_d6p_receipt_commitments.contains("receipt-required"));
        assert_ne!(
            blocked.closure_identity_commitment,
            complete.closure_identity_commitment
        );
    }

    #[test]
    fn golden_vector_recipes_execute_against_declared_fixtures() {
        let corpus: GoldenCorpusV1 = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/d6x_qualified_closure_golden_vectors.json"
        )))
        .expect("D6X golden vector corpus must parse");

        for vector in &corpus.vectors {
            let (projection, environment, derivation_profile, profile) =
                execute_golden_recipe(&corpus, vector);
            let closure = compute_dependency_closure(
                &projection,
                &environment,
                &derivation_profile,
                &profile,
            )
            .unwrap_or_else(|| panic!("golden recipe must produce a closure: {}", vector.id));
            assert_golden_vector(&corpus, &vector.id, &closure);
        }
    }

    #[test]
    fn golden_vector_hashes_match_reference_model() {
        let corpus: GoldenCorpusV1 = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/d6x_qualified_closure_golden_vectors.json"
        )))
        .expect("D6X golden vector corpus must parse");

        let (baseline, environment, derivation_profile) = projection(false);

        assert_golden_vector(
            &corpus,
            "baseline-complete",
            &compute_dependency_closure(
                &baseline,
                &environment,
                &derivation_profile,
                &profile(BTreeSet::new()),
            )
            .unwrap(),
        );

        let mut edge_free_profile = profile(BTreeSet::new());
        edge_free_profile.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Provenance,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::Any,
        }]
        .into_iter()
        .collect();
        assert_golden_vector(
            &corpus,
            "edge-free-complete",
            &compute_dependency_closure(
                &baseline,
                &environment,
                &derivation_profile,
                &edge_free_profile,
            )
            .unwrap(),
        );

        assert_golden_vector(
            &corpus,
            "missing-required-node",
            &compute_dependency_closure(
                &baseline,
                &environment,
                &derivation_profile,
                &profile(["missing".into()].into_iter().collect()),
            )
            .unwrap(),
        );

        let mut cyclic = baseline.clone();
        cyclic.edges.insert("e2".into(), QualifiedEdgeV1 {
            edge_id: "e2".into(),
            from_node_id: "dep".into(),
            to_node_id: "root".into(),
            kind: ClaimGraphEdgeKindV1::Supports,
            edge_commitment: "edge-e2".into(),
            claim_ceiling: D6X_CLAIM_CEILING.into(),
        });
        assert_golden_vector(
            &corpus,
            "cyclic-complete",
            &compute_dependency_closure(
                &cyclic,
                &environment,
                &derivation_profile,
                &profile(BTreeSet::new()),
            )
            .unwrap(),
        );

        let mut historical = baseline.clone();
        historical.nodes.get_mut("dep").unwrap().historical_only = true;
        let mut current_only = profile(BTreeSet::new());
        current_only.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::CurrentOnly,
        }]
        .into_iter()
        .collect();
        assert_golden_vector(
            &corpus,
            "currentness-historical",
            &compute_dependency_closure(
                &historical,
                &environment,
                &derivation_profile,
                &current_only,
            )
            .unwrap(),
        );

        let mut frontier_mismatch = baseline.clone();
        frontier_mismatch.nodes.get_mut("dep").unwrap().current_frontier_root =
            Some("frontier-old".into());
        assert_golden_vector(
            &corpus,
            "currentness-frontier-mismatch",
            &compute_dependency_closure(
                &frontier_mismatch,
                &environment,
                &derivation_profile,
                &current_only,
            )
            .unwrap(),
        );

        let mut frontier_omitted = baseline.clone();
        frontier_omitted.nodes.get_mut("dep").unwrap().current_frontier_root = None;
        assert_golden_vector(
            &corpus,
            "currentness-frontier-omitted",
            &compute_dependency_closure(
                &frontier_omitted,
                &environment,
                &derivation_profile,
                &current_only,
            )
            .unwrap(),
        );

        let mut node_limit = profile(BTreeSet::new());
        node_limit.max_nodes = 1;
        node_limit.max_edges = 1;
        assert_golden_vector(
            &corpus,
            "resource-node-limit",
            &compute_dependency_closure(
                &baseline,
                &environment,
                &derivation_profile,
                &node_limit,
            )
            .unwrap(),
        );

        let mut edge_limited = cyclic.clone();
        let mut edge_limit_profile = profile(BTreeSet::new());
        edge_limit_profile.max_nodes = 8;
        edge_limit_profile.max_edges = 1;
        assert_golden_vector(
            &corpus,
            "resource-edge-limit",
            &compute_dependency_closure(
                &edge_limited,
                &environment,
                &derivation_profile,
                &edge_limit_profile,
            )
            .unwrap(),
        );
    }

    #[test]
    fn source_snapshot_substitution_is_rejected_at_d6x_boundary() {
        let (baseline, environment, derivation_profile) = projection(false);
        let before = compute_dependency_closure(
            &baseline,
            &environment,
            &derivation_profile,
            &profile(BTreeSet::new()),
        )
        .expect("baseline projection must yield a closure");

        let mut changed = baseline.clone();
        changed.source_dkg_snapshot_commitment = "different-snapshot".into();
        let after = compute_dependency_closure(
            &changed,
            &environment,
            &derivation_profile,
            &profile(BTreeSet::new()),
        );

        assert_eq!(
            before.source_dkg_snapshot_commitment,
            environment
                .dependency_snapshot_root
                .clone()
                .expect("baseline environment must bind a dependency snapshot")
        );
        assert!(
            after.is_none(),
            "D6X must reject a projection whose source snapshot commitment diverges from the bound environment"
        );
    }

    #[test]
    fn closure_certificate_commitment_covers_every_semantic_field() {
        let (projection, environment, derivation_profile) = projection(false);
        let baseline = compute_dependency_closure(
            &projection,
            &environment,
            &derivation_profile,
            &profile(BTreeSet::new()),
        )
        .unwrap();
        let expected = baseline.recompute();

        let mut mutated = baseline.clone();
        mutated.schema_version = "D6X-2".into();
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.algorithm_version = "D6X-CLOSURE-5".into();
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.closure_profile_commitment = "profile-other".into();
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.source_dkg_snapshot_commitment = "snapshot-other".into();
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.projection_commitment = "projection-other".into();
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.closure_identity_commitment = "identity-other".into();
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.semantic_environment_commitment = "environment-other".into();
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.derivation_profile_commitment = "derivation-profile-other".into();
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.root_node_ids.insert("another-root".into());
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.included_node_ids.insert("another-node".into());
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.included_node_commitments.insert("commit-another-node".into());
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.included_nodes.insert("another-node".into(), "commit-another-node".into());
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.included_d6p_receipt_commitments.insert("receipt-other".into());
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.included_edge_commitments.insert("edge-other".into());
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.included_edges.insert(
            "edge-other".into(),
            ("root".into(), "dep".into(), ClaimGraphEdgeKindV1::Supports, "edge-other".into()),
        );
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.dependencies.insert(SemanticDependencyReferenceV1::node("extra", Some("commit-extra".into())));
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.missing_dependency_ids.insert("missing-other".into());
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.missing_dependencies.insert(SemanticDependencyReferenceV1::node("missing-other", None));
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.dependency_resolutions.insert(
            SemanticDependencyReferenceV1::node("extra", Some("commit-extra".into())),
            SemanticDependencyResolutionV1::Present,
        );
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.resolution_evidence.insert(
            SemanticDependencyReferenceV1::node("root", Some("commit-root".into())),
            SemanticDependencyResolutionEvidenceV1 {
                retrieval_reference: Some("other-source".into()),
                observed_commitment: None,
                qualification_context_commitment: None,
            },
        );
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.status = DependencyClosureStatusV1::BlockedResourceLimit;
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.cycle_detected = !mutated.cycle_detected;
        assert_ne!(expected, mutated.recompute());

        let mut mutated = baseline.clone();
        mutated.claim_ceiling = "different-claim-ceiling".into();
        assert_ne!(expected, mutated.recompute());

        // The stored commitment itself is excluded from its own preimage.
        let mut mutated = baseline;
        mutated.commitment = "different-commitment".into();
        assert_eq!(expected, mutated.recompute());
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
    fn stale_currentness_survives_target_node_dequeue() {
        let (mut a, e, d) = projection(false);
        a.nodes.get_mut("dep").unwrap().historical_only = true;

        let mut p = profile(BTreeSet::new());
        p.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::CurrentOnly,
        }]
        .into_iter()
        .collect();

        let c = compute_dependency_closure(&a, &e, &d, &p).unwrap();
        let dep = SemanticDependencyReferenceV1::node("dep", Some("commit-dep".into()));

        assert_eq!(
            c.dependency_resolutions.get(&dep),
            Some(&SemanticDependencyResolutionV1::Stale)
        );
        assert_eq!(c.status, DependencyClosureStatusV1::BlockedCurrentness);
        assert!(c.valid());
    }

    #[test]
    fn frontier_mismatch_marks_selected_dependency_stale() {
        let (mut a, e, d) = projection(false);
        a.nodes.get_mut("dep").unwrap().current_frontier_root = Some("frontier-old".into());

        let mut p = profile(BTreeSet::new());
        p.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::CurrentOnly,
        }]
        .into_iter()
        .collect();

        let c = compute_dependency_closure(&a, &e, &d, &p).unwrap();
        let dep = SemanticDependencyReferenceV1::node("dep", Some("commit-dep".into()));

        assert_eq!(
            c.dependency_resolutions.get(&dep),
            Some(&SemanticDependencyResolutionV1::Stale)
        );
        assert_eq!(c.status, DependencyClosureStatusV1::BlockedCurrentness);
        assert!(c.valid());
    }

    #[test]
    fn missing_frontier_metadata_blocks_current_only_qualification() {
        let (mut a, e, d) = projection(false);
        a.nodes.get_mut("dep").unwrap().current_frontier_root = None;

        let mut p = profile(BTreeSet::new());
        p.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::CurrentOnly,
        }]
        .into_iter()
        .collect();

        let c = compute_dependency_closure(&a, &e, &d, &p).unwrap();
        let dep = SemanticDependencyReferenceV1::node("dep", Some("commit-dep".into()));

        assert_eq!(
            c.dependency_resolutions.get(&dep),
            Some(&SemanticDependencyResolutionV1::Stale)
        );
        assert_eq!(c.status, DependencyClosureStatusV1::BlockedCurrentness);
        assert!(c.valid());
    }

    #[test]
    fn frontier_match_is_required_for_current_only_but_not_any() {
        let (mut a, e, d) = projection(false);
        a.nodes.get_mut("dep").unwrap().current_frontier_root = Some("frontier-old".into());

        let mut any = profile(BTreeSet::new());
        any.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::Any,
        }]
        .into_iter()
        .collect();

        let complete = compute_dependency_closure(&a, &e, &d, &any).unwrap();
        assert_eq!(complete.status, DependencyClosureStatusV1::Complete);

        let mut current = any.clone();
        current.rules = [DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::CurrentOnly,
        }]
        .into_iter()
        .collect();

        let blocked = compute_dependency_closure(&a, &e, &d, &current).unwrap();
        assert_eq!(blocked.status, DependencyClosureStatusV1::BlockedCurrentness);
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
    }
}
