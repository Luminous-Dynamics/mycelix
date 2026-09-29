//! D6S canonical derivation receipt reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! This module makes the D6S boundary executable without making the DKG a
//! truth oracle. A qualified projection is explicitly committed to its nodes,
//! edges, semantic environment, derivation profile, and any current-finality
//! qualification it consumes. Canonical bytes are deterministic; SHA-256 is
//! used only as an integrity commitment and does not confer semantic authority.

use crate::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
use crate::finality_eligibility_composition::{
    CurrentFinalityEligibilityReceiptV1, FinalityEligibilityDispositionV1,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const D6S_CLAIM_CEILING: &str =
    "ReferenceModelOnly; canonical derivation integrity semantics only; no truth, causality, authority, or actuation claim.";
pub const D6S_REFERENCE_CANONICALIZATION_VERSION: &str = "D6S-RUST-REF-1";

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|b| format!("{b:02x}")).collect()
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticEnvironmentV1 {
    pub semantic_profile_id: String,
    pub semantic_profile_version: String,
    pub current_frontier_root: Option<String>,
    pub d6p_eligibility_context_root: Option<String>,
    pub d6n_observer_context_root: Option<String>,
    pub d6o_lifecycle_context_root: Option<String>,
    pub membership_authority_scope_root: Option<String>,
    pub dependency_snapshot_root: Option<String>,
    pub historical_cutoff: Option<u64>,
    pub policy_version: String,
    pub claim_ceiling: String,
}

impl SemanticEnvironmentV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.semantic_profile_id)
            && non_empty(&self.semantic_profile_version)
            && non_empty(&self.policy_version)
            && self.claim_ceiling == D6S_CLAIM_CEILING
            && [
                &self.current_frontier_root,
                &self.d6p_eligibility_context_root,
                &self.d6n_observer_context_root,
                &self.d6o_lifecycle_context_root,
                &self.membership_authority_scope_root,
                &self.dependency_snapshot_root,
            ]
            .iter()
            .all(|v| v.as_deref().map_or(true, non_empty))
    }

    pub fn commitment(&self) -> String {
        canonical_sha256(self)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DerivationProfileV1 {
    pub profile_id: String,
    pub version: String,
    pub rule_ids: BTreeSet<String>,
    pub permits_recursive_fixpoint: bool,
    pub claim_ceiling: String,
}

impl DerivationProfileV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.profile_id)
            && non_empty(&self.version)
            && !self.rule_ids.is_empty()
            && self.claim_ceiling == D6S_CLAIM_CEILING
            && (!self.permits_recursive_fixpoint || self.rule_ids.iter().any(|r| r == "recursive-fixpoint-v1"))
    }

    pub fn commitment(&self) -> String {
        canonical_sha256(self)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualifiedNodeV1 {
    pub node_id: String,
    pub kind: ClaimGraphNodeKindV1,
    pub node_commitment: String,
    pub historical_only: bool,
    pub current_frontier_root: Option<String>,
    pub claim_ceiling: String,
}

impl QualifiedNodeV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.node_id)
            && non_empty(&self.node_commitment)
            && self.claim_ceiling == D6S_CLAIM_CEILING
            && self.current_frontier_root.as_deref().map_or(true, non_empty)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualifiedEdgeV1 {
    pub edge_id: String,
    pub from_node_id: String,
    pub to_node_id: String,
    pub kind: ClaimGraphEdgeKindV1,
    pub edge_commitment: String,
    pub claim_ceiling: String,
}

impl QualifiedEdgeV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.edge_id)
            && non_empty(&self.from_node_id)
            && non_empty(&self.to_node_id)
            && self.from_node_id != self.to_node_id
            && non_empty(&self.edge_commitment)
            && self.claim_ceiling == D6S_CLAIM_CEILING
    }

    fn is_derivation_semantic(&self) -> bool {
        matches!(self.kind, ClaimGraphEdgeKindV1::DerivedFrom | ClaimGraphEdgeKindV1::Supports)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualifiedProjectionV1 {
    pub projection_id: String,
    pub projection_version: String,
    pub canonicalization_version: String,
    pub source_dkg_snapshot_commitment: String,
    pub nodes: BTreeMap<String, QualifiedNodeV1>,
    pub edges: BTreeMap<String, QualifiedEdgeV1>,
    pub d6p_current_receipt_commitments: BTreeSet<String>,
    pub d6n_context_commitment: Option<String>,
    pub d6o_context_commitment: Option<String>,
    pub semantic_environment_commitment: String,
    pub derivation_profile_commitment: String,
    pub claim_ceiling: String,
}

impl QualifiedProjectionV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.projection_id)
            && non_empty(&self.projection_version)
            && self.canonicalization_version == D6S_REFERENCE_CANONICALIZATION_VERSION
            && non_empty(&self.source_dkg_snapshot_commitment)
            && !self.nodes.is_empty()
            && !self.edges.is_empty()
            && non_empty(&self.semantic_environment_commitment)
            && non_empty(&self.derivation_profile_commitment)
            && self.claim_ceiling == D6S_CLAIM_CEILING
            && self.nodes.iter().all(|(id, node)| id == &node.node_id && node.structurally_valid())
            && self.edges.iter().all(|(id, edge)| id == &edge.edge_id && edge.structurally_valid())
            && self.d6p_current_receipt_commitments.iter().all(|v| non_empty(v))
            && self.d6n_context_commitment.as_deref().map_or(true, non_empty)
            && self.d6o_context_commitment.as_deref().map_or(true, non_empty)
    }

    pub fn dangling_edge_ids(&self) -> BTreeSet<String> {
        self.edges.values()
            .filter(|e| !self.nodes.contains_key(&e.from_node_id) || !self.nodes.contains_key(&e.to_node_id))
            .map(|e| e.edge_id.clone())
            .collect()
    }

    pub fn derivation_cycle_exists(&self) -> bool {
        fn visit(
            id: &str,
            adjacency: &BTreeMap<String, BTreeSet<String>>,
            visiting: &mut BTreeSet<String>,
            visited: &mut BTreeSet<String>,
        ) -> bool {
            if visiting.contains(id) { return true; }
            if visited.contains(id) { return false; }
            visiting.insert(id.to_owned());
            if adjacency.get(id).is_some_and(|next| next.iter().any(|child| visit(child, adjacency, visiting, visited))) {
                return true;
            }
            visiting.remove(id);
            visited.insert(id.to_owned());
            false
        }

        let mut adjacency = BTreeMap::<String, BTreeSet<String>>::new();
        for edge in self.edges.values().filter(|e| e.is_derivation_semantic()) {
            adjacency.entry(edge.from_node_id.clone()).or_default().insert(edge.to_node_id.clone());
        }
        let mut visiting = BTreeSet::new();
        let mut visited = BTreeSet::new();
        adjacency.keys().any(|id| visit(id, &adjacency, &mut visiting, &mut visited))
    }

    pub fn incompatible_edge_ids(&self) -> BTreeSet<String> {
        self.edges.values()
            .filter(|edge| {
                let (Some(from), Some(to)) = (self.nodes.get(&edge.from_node_id), self.nodes.get(&edge.to_node_id)) else {
                    return true;
                };
                !crate::evidence_claim_graph::EvidenceBundleV1::edge_type_compatible(from.kind, to.kind, edge.kind)
            })
            .map(|edge| edge.edge_id.clone())
            .collect()
    }

    pub fn commitment(&self) -> String {
        canonical_sha256(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DerivationResultStatusV1 {
    Unresolved,
    Supported,
    Disputed,
    Rejected,
    BlockedMissingEvidence,
    BlockedCurrentness,
    BlockedQualification,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CanonicalDerivationReceiptV1 {
    pub schema_version: String,
    pub projection_version: String,
    pub canonicalization_version: String,
    pub source_dkg_snapshot_commitment: String,
    pub projection_commitment: String,
    pub semantic_environment_commitment: String,
    pub derivation_profile_commitment: String,
    pub input_node_commitments: BTreeSet<String>,
    pub input_edge_commitments: BTreeSet<String>,
    pub d6p_current_receipt_commitments: BTreeSet<String>,
    pub result_status: DerivationResultStatusV1,
    pub result_commitment: String,
    pub contradiction_preserved: bool,
    pub unresolved_preserved: bool,
    pub claim_ceiling: String,
    pub receipt_commitment: String,
}

impl CanonicalDerivationReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.schema_version)
            && non_empty(&self.projection_version)
            && self.canonicalization_version == D6S_REFERENCE_CANONICALIZATION_VERSION
            && non_empty(&self.source_dkg_snapshot_commitment)
            && non_empty(&self.projection_commitment)
            && non_empty(&self.semantic_environment_commitment)
            && non_empty(&self.derivation_profile_commitment)
            && !self.input_node_commitments.is_empty()
            && !self.input_edge_commitments.is_empty()
            && self.input_node_commitments.iter().all(|v| non_empty(v))
            && self.input_edge_commitments.iter().all(|v| non_empty(v))
            && self.d6p_current_receipt_commitments.iter().all(|v| non_empty(v))
            && non_empty(&self.result_commitment)
            && non_empty(&self.receipt_commitment)
            && self.claim_ceiling == D6S_CLAIM_CEILING
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        serde_json::to_vec(self).expect("D6S reference values are serializable")
    }

    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.receipt_commitment.clear();
        sha256_hex(&unsigned.canonical_bytes())
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.receipt_commitment == self.recomputed_commitment()
    }
}

pub fn canonical_sha256<T: Serialize>(value: &T) -> String {
    let bytes = serde_json::to_vec(value).expect("D6S reference values are serializable");
    sha256_hex(&bytes)
}

pub fn commitment_set_digest(values: &BTreeSet<String>) -> String {
    canonical_sha256(values)
}

fn result_flags_are_consistent(
    status: DerivationResultStatusV1,
    contradiction_preserved: bool,
    unresolved_preserved: bool,
) -> bool {
    match status {
        DerivationResultStatusV1::Supported => !contradiction_preserved && !unresolved_preserved,
        DerivationResultStatusV1::Disputed => contradiction_preserved,
        DerivationResultStatusV1::Unresolved
        | DerivationResultStatusV1::BlockedMissingEvidence
        | DerivationResultStatusV1::BlockedCurrentness
        | DerivationResultStatusV1::BlockedQualification => unresolved_preserved,
        DerivationResultStatusV1::Rejected => true,
    }
}

fn projection_context_matches_environment(
    projection: &QualifiedProjectionV1,
    environment: &SemanticEnvironmentV1,
) -> bool {
    projection.d6n_context_commitment == environment.d6n_observer_context_root
        && projection.d6o_context_commitment == environment.d6o_lifecycle_context_root
        && match environment.d6p_eligibility_context_root.as_deref() {
            Some(expected) => commitment_set_digest(&projection.d6p_current_receipt_commitments) == expected,
            None => projection.d6p_current_receipt_commitments.is_empty(),
        }
}

pub fn build_canonical_receipt(
    projection: &QualifiedProjectionV1,
    environment: &SemanticEnvironmentV1,
    profile: &DerivationProfileV1,
    current_receipts: &[CurrentFinalityEligibilityReceiptV1],
    result_status: DerivationResultStatusV1,
    result_commitment: String,
    contradiction_preserved: bool,
    unresolved_preserved: bool,
) -> Option<CanonicalDerivationReceiptV1> {
    if !environment.structurally_valid()
        || !profile.structurally_valid()
        || !projection.structurally_valid()
        || projection.semantic_environment_commitment != environment.commitment()
        || projection.derivation_profile_commitment != profile.commitment()
        || !projection_context_matches_environment(projection, environment)
        || !projection.dangling_edge_ids().is_empty()
        || !projection.incompatible_edge_ids().is_empty()
        || (projection.derivation_cycle_exists() && !profile.permits_recursive_fixpoint)
        || result_commitment.trim().is_empty()
        || !result_flags_are_consistent(result_status, contradiction_preserved, unresolved_preserved)
        || (matches!(result_status, DerivationResultStatusV1::Supported)
            && (environment.current_frontier_root.is_none()
                || projection.nodes.values().any(|node| {
                    node.historical_only || node.current_frontier_root != environment.current_frontier_root
                })
                || projection.d6p_current_receipt_commitments.is_empty()
                || !projection.d6p_current_receipt_commitments.iter().all(|expected| {
                    current_receipts.iter().any(|receipt| current_receipt_is_bound(
                        receipt,
                        expected,
                        environment,
                    ))
                })))
    {
        return None;
    }

    let input_node_commitments = projection.nodes.values().map(|n| n.node_commitment.clone()).collect();
    let input_edge_commitments = projection.edges.values().map(|e| e.edge_commitment.clone()).collect();

    let mut receipt = CanonicalDerivationReceiptV1 {
        schema_version: "D6S-1".into(),
        projection_version: projection.projection_version.clone(),
        canonicalization_version: projection.canonicalization_version.clone(),
        source_dkg_snapshot_commitment: projection.source_dkg_snapshot_commitment.clone(),
        projection_commitment: projection.commitment(),
        semantic_environment_commitment: environment.commitment(),
        derivation_profile_commitment: profile.commitment(),
        input_node_commitments,
        input_edge_commitments,
        d6p_current_receipt_commitments: projection.d6p_current_receipt_commitments.clone(),
        result_status,
        result_commitment,
        contradiction_preserved,
        unresolved_preserved,
        claim_ceiling: D6S_CLAIM_CEILING.into(),
        receipt_commitment: String::new(),
    };
    receipt.receipt_commitment = receipt.recomputed_commitment();
    Some(receipt)
}

pub fn current_receipt_is_bound(
    receipt: &CurrentFinalityEligibilityReceiptV1,
    expected_commitment: &str,
    environment: &SemanticEnvironmentV1,
) -> bool {
    receipt.structurally_valid()
        && matches!(receipt.disposition, FinalityEligibilityDispositionV1::EligibleCurrent)
        && receipt.receipt_commitment == expected_commitment
        && receipt.claim_ceiling
            == crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING
        && environment
            .current_frontier_root
            .as_deref()
            .is_some_and(|root| root == receipt.current_frontier_root)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn env() -> SemanticEnvironmentV1 {
        SemanticEnvironmentV1 {
            semantic_profile_id: "integral".into(),
            semantic_profile_version: "1".into(),
            current_frontier_root: Some("frontier-1".into()),
            d6p_eligibility_context_root: Some("d6p-1".into()),
            d6n_observer_context_root: Some("d6n-1".into()),
            d6o_lifecycle_context_root: Some("d6o-1".into()),
            membership_authority_scope_root: Some("membership-1".into()),
            dependency_snapshot_root: Some("deps-1".into()),
            historical_cutoff: Some(100),
            policy_version: "policy-1".into(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        }
    }

    fn profile() -> DerivationProfileV1 {
        DerivationProfileV1 {
            profile_id: "support-v1".into(),
            version: "1".into(),
            rule_ids: ["bound-support-v1".into()].into_iter().collect(),
            permits_recursive_fixpoint: false,
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        }
    }

    fn d6p_receipt() -> CurrentFinalityEligibilityReceiptV1 {
        CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "d6p-1".into(), effect_id: "effect".into(), effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "generation".into(), route_id: "route".into(), provider_id: "provider".into(),
            provider_operation_id: "operation".into(), provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(), observation_set_id: "set".into(), observation_set_commitment: "set-c".into(),
            d6n_assessment_commitment: "d6n-c".into(), witness_eligibility_ids: ["w".into()].into_iter().collect(),
            observer_generation_ids: ["g".into()].into_iter().collect(), current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life".into(), eligible_independent_count: 1, preserved_contradictory_count: 0,
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent, qualification_transition_id: "t".into(),
            receipt_commitment: "d6p-receipt-1".into(),
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        }
    }

    fn projection() -> QualifiedProjectionV1 {
        let mut nodes = BTreeMap::new();
        nodes.insert("e".into(), QualifiedNodeV1 {
            node_id: "e".into(), kind: ClaimGraphNodeKindV1::Evidence,
            node_commitment: "node-e".into(), historical_only: false,
            current_frontier_root: Some("frontier-1".into()), claim_ceiling: D6S_CLAIM_CEILING.into(),
        });
        nodes.insert("a".into(), QualifiedNodeV1 {
            node_id: "a".into(), kind: ClaimGraphNodeKindV1::Assessment,
            node_commitment: "node-a".into(), historical_only: false,
            current_frontier_root: Some("frontier-1".into()), claim_ceiling: D6S_CLAIM_CEILING.into(),
        });
        nodes.insert("c".into(), QualifiedNodeV1 {
            node_id: "c".into(), kind: ClaimGraphNodeKindV1::Conclusion,
            node_commitment: "node-c".into(), historical_only: false,
            current_frontier_root: Some("frontier-1".into()), claim_ceiling: D6S_CLAIM_CEILING.into(),
        });
        let mut edges = BTreeMap::new();
        edges.insert("p".into(), QualifiedEdgeV1 {
            edge_id: "p".into(), from_node_id: "e".into(), to_node_id: "a".into(),
            kind: ClaimGraphEdgeKindV1::Provenance, edge_commitment: "edge-p".into(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        });
        edges.insert("s".into(), QualifiedEdgeV1 {
            edge_id: "s".into(), from_node_id: "a".into(), to_node_id: "c".into(),
            kind: ClaimGraphEdgeKindV1::Supports, edge_commitment: "edge-s".into(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        });
        let e = env();
        let p = profile();
        QualifiedProjectionV1 {
            projection_id: "projection-1".into(), projection_version: "1".into(),
            canonicalization_version: D6S_REFERENCE_CANONICALIZATION_VERSION.into(),
            source_dkg_snapshot_commitment: "dkg-snapshot-1".into(),
            nodes, edges, d6p_current_receipt_commitments: ["d6p-receipt-1".into()].into_iter().collect(),
            d6n_context_commitment: Some("d6n-1".into()), d6o_context_commitment: Some("d6o-1".into()),
            semantic_environment_commitment: e.commitment(), derivation_profile_commitment: p.commitment(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        }
    }

    #[test]
    fn canonical_bytes_are_deterministic() {
        let a = projection();
        let b = projection();
        assert_eq!(a.canonical_bytes_for_test(), b.canonical_bytes_for_test());
    }

    #[test]
    fn input_substitution_changes_projection_commitment() {
        let mut p = projection();
        let before = p.commitment();
        p.nodes.get_mut("e").unwrap().node_commitment = "node-e-replaced".into();
        assert_ne!(before, p.commitment());
    }

    #[test]
    fn environment_substitution_changes_receipt() {
        let p = projection();
        let profile = profile();
        let first = build_canonical_receipt(&p, &env(), &profile, &[d6p_receipt()], DerivationResultStatusV1::Supported, "result-1".into(), false, false).unwrap();

        let mut changed_env = env();
        changed_env.policy_version = "policy-2".into();
        let mut changed_projection = p.clone();
        changed_projection.semantic_environment_commitment = changed_env.commitment();
        let second = build_canonical_receipt(&changed_projection, &changed_env, &profile, &[d6p_receipt()], DerivationResultStatusV1::Supported, "result-1".into(), false, false).unwrap();

        assert_ne!(first.receipt_commitment, second.receipt_commitment);
    }

    #[test]
    fn profile_substitution_changes_receipt() {
        let p = projection();
        let e = env();
        let first = build_canonical_receipt(&p, &e, &profile(), &[d6p_receipt()], DerivationResultStatusV1::Supported, "result-1".into(), false, false).unwrap();
        let mut changed = profile();
        changed.version = "2".into();
        let mut changed_projection = p.clone();
        changed_projection.derivation_profile_commitment = changed.commitment();
        let second = build_canonical_receipt(&changed_projection, &e, &changed, &[d6p_receipt()], DerivationResultStatusV1::Supported, "result-1".into(), false, false).unwrap();
        assert_ne!(first.receipt_commitment, second.receipt_commitment);
    }

    #[test]
    fn missing_d6p_receipt_is_not_current_qualification() {
        let p = projection();
        assert!(build_canonical_receipt(&p, &env(), &profile(), &[], DerivationResultStatusV1::Supported, "result-1".into(), false, false).is_none());
    }

    #[test]
    fn current_d6p_receipt_can_qualify_only_at_the_exact_frontier() {
        let p = projection();
        assert!(build_canonical_receipt(
            &p, &env(), &profile(), &[d6p_receipt()],
            DerivationResultStatusV1::Supported, "result-1".into(), false, false
        ).is_some());

        let mut stale = d6p_receipt();
        stale.current_frontier_root = "frontier-0".into();
        assert!(build_canonical_receipt(
            &p, &env(), &profile(), &[stale],
            DerivationResultStatusV1::Supported, "result-1".into(), false, false
        ).is_none());
    }

    #[test]
    fn derivation_cycles_are_rejected_but_provenance_cycles_are_not() {
        let mut p = projection();
        p.nodes.insert("x".into(), QualifiedNodeV1 {
            node_id: "x".into(), kind: ClaimGraphNodeKindV1::Assessment,
            node_commitment: "node-x".into(), historical_only: false,
            current_frontier_root: Some("frontier-1".into()), claim_ceiling: D6S_CLAIM_CEILING.into(),
        });
        p.edges.insert("cycle-a".into(), QualifiedEdgeV1 {
            edge_id: "cycle-a".into(), from_node_id: "a".into(), to_node_id: "x".into(),
            kind: ClaimGraphEdgeKindV1::Supports, edge_commitment: "edge-a".into(), claim_ceiling: D6S_CLAIM_CEILING.into(),
        });
        p.edges.insert("cycle-b".into(), QualifiedEdgeV1 {
            edge_id: "cycle-b".into(), from_node_id: "x".into(), to_node_id: "a".into(),
            kind: ClaimGraphEdgeKindV1::Provenance, edge_commitment: "edge-b".into(), claim_ceiling: D6S_CLAIM_CEILING.into(),
        });
        assert!(!p.derivation_cycle_exists());
        p.edges.get_mut("cycle-b").unwrap().kind = ClaimGraphEdgeKindV1::Supports;
        assert!(p.derivation_cycle_exists());
    }

    #[test]
    fn historical_input_cannot_be_promoted_to_supported_current_result() {
        let mut p = projection();
        p.nodes.get_mut("e").unwrap().historical_only = true;
        assert!(build_canonical_receipt(
            &p, &env(), &profile(), &[d6p_receipt()],
            DerivationResultStatusV1::Supported, "result-1".into(), false, false
        ).is_none());
    }

    #[test]
    fn projection_context_must_match_the_semantic_environment() {
        let mut p = projection();
        p.d6n_context_commitment = Some("different-d6n".into());
        assert!(build_canonical_receipt(
            &p, &env(), &profile(), &[d6p_receipt()],
            DerivationResultStatusV1::Supported, "result-1".into(), false, false
        ).is_none());
    }

    #[test]
    fn result_status_must_match_preserved_conflict_state() {
        assert!(result_flags_are_consistent(
            DerivationResultStatusV1::Disputed, true, false
        ));
        assert!(!result_flags_are_consistent(
            DerivationResultStatusV1::Disputed, false, false
        ));
        assert!(result_flags_are_consistent(
            DerivationResultStatusV1::BlockedMissingEvidence, false, true
        ));
        assert!(!result_flags_are_consistent(
            DerivationResultStatusV1::Supported, true, false
        ));
    }

    #[test]
    fn source_dkg_snapshot_and_canonicalization_version_are_receipt_inputs() {
        let p = projection();
        let e = env();
        let first = build_canonical_receipt(
            &p, &e, &profile(), &[d6p_receipt()],
            DerivationResultStatusV1::Supported, "result-1".into(), false, false
        ).unwrap();

        let mut changed = p.clone();
        changed.source_dkg_snapshot_commitment = "dkg-snapshot-2".into();
        let second = build_canonical_receipt(
            &changed, &e, &profile(), &[d6p_receipt()],
            DerivationResultStatusV1::Supported, "result-1".into(), false, false
        ).unwrap();
        assert_ne!(first.receipt_commitment, second.receipt_commitment);

        let mut changed = p;
        changed.canonicalization_version = "D6S-RUST-REF-2".into();
        assert!(!changed.structurally_valid());
    }

    #[test]
    fn receipt_commitment_is_self_consistent() {
        let r = build_canonical_receipt(&projection(), &env(), &profile(), &[d6p_receipt()], DerivationResultStatusV1::Supported, "result-1".into(), false, false).unwrap();
        assert!(r.commitment_matches());
    }

    #[test]
    fn contradiction_and_unresolved_are_explicit() {
        let r = build_canonical_receipt(&projection(), &env(), &profile(), &[], DerivationResultStatusV1::Disputed, "result-1".into(), true, false).unwrap();
        assert!(r.contradiction_preserved);
        assert!(!r.unresolved_preserved);
        assert!(r.commitment_matches());
    }

    #[test]
    fn d6p_binding_is_exact_and_non_authorizing() {
        let r = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "r".into(), effect_id: "effect".into(), effect_lineage_id: "lineage".into(),
            lifecycle_generation_id: "generation".into(), route_id: "route".into(), provider_id: "provider".into(),
            provider_operation_id: "operation".into(), provider_profile_root: "provider-profile".into(),
            semantic_environment_root: "env".into(), observation_set_id: "set".into(), observation_set_commitment: "set-c".into(),
            d6n_assessment_commitment: "d6n-c".into(), witness_eligibility_ids: ["w".into()].into_iter().collect(),
            observer_generation_ids: ["g".into()].into_iter().collect(), current_frontier_root: "frontier".into(),
            lifecycle_profile_id: "life".into(), eligible_independent_count: 1, preserved_contradictory_count: 0,
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent, qualification_transition_id: "t".into(),
            receipt_commitment: "d6p-commitment".into(),
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        assert!(current_receipt_is_bound(&r, "d6p-commitment", &env()));
        assert!(!current_receipt_is_bound(&r, "different", &env()));
    }
}

trait ProjectionCanonicalTestBytes {
    fn canonical_bytes_for_test(&self) -> Vec<u8>;
}
impl ProjectionCanonicalTestBytes for QualifiedProjectionV1 {
    fn canonical_bytes_for_test(&self) -> Vec<u8> {
        serde_json::to_vec(self).expect("projection is serializable")
    }
}
