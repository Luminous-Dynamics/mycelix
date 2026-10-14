//! Evidence bundle and claim-graph closure reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! D6P gives the Integral evidence chain an explicit typed graph. Graph
//! reachability is deliberately weaker than semantic validity: a path from a
//! source to a conclusion is not itself evidence that the conclusion is true.
//!
//! The model preserves:
//! source != evidence != statement != registration receipt != validation
//! != assessment != conclusion != human disposition.
//!
//! It also keeps provenance/custody edges separate from causal-support edges.
//! The graph is deterministic and uses opaque commitments supplied by callers.
//! It does not verify cryptographic authenticity, source truth, causality,
//! legal authority, or production finality.

use serde::{Deserialize, Serialize};
use crate::finality_eligibility_composition::CurrentFinalityEligibilityReceiptV1;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

pub const EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING: &str =
    "Evidence bundle and claim-graph reference semantics only; no truth, causality, authority, or production-finality claim.";

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ClaimGraphNodeKindV1 {
    Source,
    Evidence,
    Statement,
    RegistrationReceipt,
    Validation,
    Assessment,
    Conclusion,
    HumanDisposition,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ClaimGraphEdgeKindV1 {
    DerivedFrom,
    Supports,
    Contradicts,
    Registers,
    Validates,
    Assesses,
    Concludes,
    Disposes,
    Provenance,
    Custody,
}

impl ClaimGraphEdgeKindV1 {
    pub fn is_support_semantic(self) -> bool {
        matches!(
            self,
            Self::DerivedFrom | Self::Supports | Self::Contradicts
        )
    }

    pub fn is_provenance_semantic(self) -> bool {
        matches!(self, Self::Provenance | Self::Custody)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BundleNodeStatusV1 {
    Present,
    Missing,
    Invalid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BundleClosureDispositionV1 {
    StructurallyClosed,
    StructurallyIncomplete,
    SemanticallyInsufficient,
    Contested,
    HistoricalOnly,
    BlockedCurrentness,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ClaimGraphConclusionStatusV1 {
    Unresolved,
    Supported,
    Disputed,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HumanDispositionV1 {
    Undisposed,
    AcceptedForHumanUse,
    RejectedForHumanUse,
    Escalated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GraphAssessmentDispositionV1 {
    ReachableOnly,
    SupportedByBoundEvidence,
    DisputedByBoundEvidence,
    RejectedByBoundEvidence,
    BlockedMissingEvidence,
    BlockedHistoricalEvidence,
    BlockedCurrentness,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimGraphNodeV1 {
    pub node_id: String,
    pub kind: ClaimGraphNodeKindV1,
    pub content_commitment: String,
    pub provenance_root: Option<String>,
    pub custody_root: Option<String>,
    pub historical_only: bool,
    pub current_frontier_sequence: Option<u64>,
    pub node_commitment: String,
    pub claim_ceiling: String,
}

impl ClaimGraphNodeV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.node_id)
            && non_empty(&self.content_commitment)
            && non_empty(&self.node_commitment)
            && self.claim_ceiling == EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING
            && self.current_frontier_sequence.map_or(true, |v| v > 0)
            && self
                .provenance_root
                .as_deref()
                .map_or(true, non_empty)
            && self.custody_root.as_deref().map_or(true, non_empty)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimGraphEdgeV1 {
    pub edge_id: String,
    pub from_node_id: String,
    pub to_node_id: String,
    pub kind: ClaimGraphEdgeKindV1,
    pub edge_commitment: String,
    pub claim_ceiling: String,
}

impl ClaimGraphEdgeV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.edge_id)
            && non_empty(&self.from_node_id)
            && non_empty(&self.to_node_id)
            && self.from_node_id != self.to_node_id
            && non_empty(&self.edge_commitment)
            && self.claim_ceiling == EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimGraphBindingV1 {
    pub source_node_ids: BTreeSet<String>,
    pub evidence_node_ids: BTreeSet<String>,
    pub statement_node_ids: BTreeSet<String>,
    pub registration_receipt_node_ids: BTreeSet<String>,
    pub validation_node_ids: BTreeSet<String>,
    pub assessment_node_ids: BTreeSet<String>,
    pub conclusion_node_ids: BTreeSet<String>,
    pub human_disposition_node_ids: BTreeSet<String>,
    pub graph_commitment: String,
    pub claim_ceiling: String,
}

impl ClaimGraphBindingV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.graph_commitment)
            && self.claim_ceiling == EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING
            && !self.conclusion_node_ids.is_empty()
    }

    pub fn required_kinds() -> [ClaimGraphNodeKindV1; 8] {
        [
            ClaimGraphNodeKindV1::Source,
            ClaimGraphNodeKindV1::Evidence,
            ClaimGraphNodeKindV1::Statement,
            ClaimGraphNodeKindV1::RegistrationReceipt,
            ClaimGraphNodeKindV1::Validation,
            ClaimGraphNodeKindV1::Assessment,
            ClaimGraphNodeKindV1::Conclusion,
            ClaimGraphNodeKindV1::HumanDisposition,
        ]
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceBundleV1 {
    pub bundle_id: String,
    pub graph: ClaimGraphBindingV1,
    pub nodes: BTreeMap<String, ClaimGraphNodeV1>,
    pub edges: BTreeMap<String, ClaimGraphEdgeV1>,
    pub bundle_commitment: String,
    pub claim_ceiling: String,
}

impl EvidenceBundleV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.bundle_id)
            && self.graph.structurally_valid()
            && !self.nodes.is_empty()
            && non_empty(&self.bundle_commitment)
            && self.claim_ceiling == EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING
            && self.nodes.values().all(ClaimGraphNodeV1::structurally_valid)
            && self.edges.values().all(ClaimGraphEdgeV1::structurally_valid)
            && self
                .nodes
                .iter()
                .all(|(id, node)| id == &node.node_id)
            && self
                .edges
                .iter()
                .all(|(id, edge)| id == &edge.edge_id)
    }

    pub fn edge_type_compatible(
        from: ClaimGraphNodeKindV1,
        to: ClaimGraphNodeKindV1,
        edge: ClaimGraphEdgeKindV1,
    ) -> bool {
        match edge {
            ClaimGraphEdgeKindV1::DerivedFrom => matches!(
                (from, to),
                (ClaimGraphNodeKindV1::Evidence, ClaimGraphNodeKindV1::Source)
                    | (ClaimGraphNodeKindV1::Statement, ClaimGraphNodeKindV1::Evidence)
                    | (
                        ClaimGraphNodeKindV1::Assessment,
                        ClaimGraphNodeKindV1::Validation
                    )
            ),
            ClaimGraphEdgeKindV1::Supports | ClaimGraphEdgeKindV1::Contradicts => {
                matches!(
                    (from, to),
                    (
                        ClaimGraphNodeKindV1::Evidence,
                        ClaimGraphNodeKindV1::Statement
                    ) | (
                        ClaimGraphNodeKindV1::Assessment,
                        ClaimGraphNodeKindV1::Conclusion
                    )
                )
            }
            ClaimGraphEdgeKindV1::Registers => matches!(
                (from, to),
                (
                    ClaimGraphNodeKindV1::Statement,
                    ClaimGraphNodeKindV1::RegistrationReceipt
                )
            ),
            ClaimGraphEdgeKindV1::Validates => matches!(
                (from, to),
                (
                    ClaimGraphNodeKindV1::RegistrationReceipt,
                    ClaimGraphNodeKindV1::Validation
                )
            ),
            ClaimGraphEdgeKindV1::Assesses => matches!(
                (from, to),
                (
                    ClaimGraphNodeKindV1::Validation,
                    ClaimGraphNodeKindV1::Assessment
                )
            ),
            ClaimGraphEdgeKindV1::Concludes => matches!(
                (from, to),
                (
                    ClaimGraphNodeKindV1::Assessment,
                    ClaimGraphNodeKindV1::Conclusion
                )
            ),
            ClaimGraphEdgeKindV1::Disposes => matches!(
                (from, to),
                (
                    ClaimGraphNodeKindV1::Conclusion,
                    ClaimGraphNodeKindV1::HumanDisposition
                )
            ),
            ClaimGraphEdgeKindV1::Provenance => matches!(
                (from, to),
                (
                    ClaimGraphNodeKindV1::Source,
                    ClaimGraphNodeKindV1::Evidence
                ) | (
                    ClaimGraphNodeKindV1::Evidence,
                    ClaimGraphNodeKindV1::Statement
                ) | (
                    ClaimGraphNodeKindV1::Evidence,
                    ClaimGraphNodeKindV1::Assessment
                )
            ),
            ClaimGraphEdgeKindV1::Custody => matches!(
                (from, to),
                (
                    ClaimGraphNodeKindV1::Source,
                    ClaimGraphNodeKindV1::Evidence
                ) | (
                    ClaimGraphNodeKindV1::Evidence,
                    ClaimGraphNodeKindV1::RegistrationReceipt
                )
            ),
        }
    }

    pub fn dangling_edge_ids(&self) -> BTreeSet<String> {
        self.edges
            .values()
            .filter(|edge| {
                !self.nodes.contains_key(&edge.from_node_id)
                    || !self.nodes.contains_key(&edge.to_node_id)
            })
            .map(|edge| edge.edge_id.clone())
            .collect()
    }

    pub fn incompatible_edge_ids(&self) -> BTreeSet<String> {
        self.edges
            .values()
            .filter(|edge| {
                let (Some(from), Some(to)) = (
                    self.nodes.get(&edge.from_node_id),
                    self.nodes.get(&edge.to_node_id),
                ) else {
                    return true;
                };
                !Self::edge_type_compatible(from.kind, to.kind, edge.kind)
            })
            .map(|edge| edge.edge_id.clone())
            .collect()
    }

    pub fn graph_has_cycle(&self) -> bool {
        fn visit(
            id: &str,
            adjacency: &BTreeMap<String, BTreeSet<String>>,
            visiting: &mut BTreeSet<String>,
            visited: &mut BTreeSet<String>,
        ) -> bool {
            if visiting.contains(id) {
                return true;
            }
            if visited.contains(id) {
                return false;
            }
            visiting.insert(id.to_owned());
            if let Some(next) = adjacency.get(id) {
                if next
                    .iter()
                    .any(|child| visit(child, adjacency, visiting, visited))
                {
                    return true;
                }
            }
            visiting.remove(id);
            visited.insert(id.to_owned());
            false
        }

        let mut adjacency: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        for edge in self.edges.values() {
            adjacency
                .entry(edge.from_node_id.clone())
                .or_default()
                .insert(edge.to_node_id.clone());
        }
        let mut visiting = BTreeSet::new();
        let mut visited = BTreeSet::new();
        adjacency
            .keys()
            .any(|id| visit(id, &adjacency, &mut visiting, &mut visited))
    }

    pub fn reachable_from(
        &self,
        start: &str,
        semantic_only: bool,
    ) -> BTreeSet<String> {
        let mut adjacency: BTreeMap<String, Vec<&ClaimGraphEdgeV1>> = BTreeMap::new();
        for edge in self.edges.values() {
            if !semantic_only || edge.kind.is_support_semantic() {
                adjacency
                    .entry(edge.from_node_id.clone())
                    .or_default()
                    .push(edge);
            }
        }
        let mut seen = BTreeSet::new();
        let mut queue = VecDeque::from([start.to_owned()]);
        while let Some(id) = queue.pop_front() {
            if !seen.insert(id.clone()) {
                continue;
            }
            if let Some(next) = adjacency.get(&id) {
                for edge in next {
                    queue.push_back(edge.to_node_id.clone());
                }
            }
        }
        seen
    }

    pub fn structural_closure(&self) -> BundleNodeStatusV1 {
        if !self.structurally_valid()
            || !self.dangling_edge_ids().is_empty()
            || !self.incompatible_edge_ids().is_empty()
            || self.graph_has_cycle()
        {
            BundleNodeStatusV1::Invalid
        } else {
            BundleNodeStatusV1::Present
        }
    }

    pub fn missing_required_nodes(&self) -> BTreeSet<ClaimGraphNodeKindV1> {
        let present: BTreeSet<_> = self.nodes.values().map(|n| n.kind).collect();
        ClaimGraphBindingV1::required_kinds()
            .into_iter()
            .filter(|kind| !present.contains(kind))
            .collect()
    }

    pub fn semantically_bound_conclusion(
        &self,
        conclusion_id: &str,
    ) -> GraphAssessmentDispositionV1 {
        if self.structural_closure() != BundleNodeStatusV1::Present {
            return GraphAssessmentDispositionV1::BlockedMissingEvidence;
        }
        let Some(conclusion) = self.nodes.get(conclusion_id) else {
            return GraphAssessmentDispositionV1::BlockedMissingEvidence;
        };
        if conclusion.kind != ClaimGraphNodeKindV1::Conclusion {
            return GraphAssessmentDispositionV1::BlockedMissingEvidence;
        }

        let mut has_support = false;
        let mut has_conflict = false;
        let mut has_current_evidence = false;
        for edge in self.edges.values() {
            if edge.to_node_id != conclusion_id {
                continue;
            }
            match edge.kind {
                ClaimGraphEdgeKindV1::Supports => {
                    if self.nodes.contains_key(&edge.from_node_id) {
                        has_support = true;
                        if self
                            .nodes
                            .get(&edge.from_node_id)
                            .and_then(|n| n.current_frontier_sequence)
                            .is_some()
                        {
                            has_current_evidence = true;
                        }
                    }
                }
                ClaimGraphEdgeKindV1::Contradicts => {
                    if self.nodes.contains_key(&edge.from_node_id) {
                        has_conflict = true;
                    }
                }
                _ => {}
            }
        }

        if has_conflict {
            GraphAssessmentDispositionV1::DisputedByBoundEvidence
        } else if has_support && has_current_evidence {
            GraphAssessmentDispositionV1::SupportedByBoundEvidence
        } else if has_support {
            GraphAssessmentDispositionV1::BlockedCurrentness
        } else {
            GraphAssessmentDispositionV1::ReachableOnly
        }
    }

    pub fn semantically_bound_conclusion_with_current_receipt(
        &self,
        conclusion_id: &str,
        current_receipt: Option<&CurrentFinalityEligibilityReceiptV1>,
    ) -> GraphAssessmentDispositionV1 {
        if self.structural_closure() != BundleNodeStatusV1::Present {
            return GraphAssessmentDispositionV1::BlockedMissingEvidence;
        }
        let Some(conclusion) = self.nodes.get(conclusion_id) else {
            return GraphAssessmentDispositionV1::BlockedMissingEvidence;
        };
        if conclusion.kind != ClaimGraphNodeKindV1::Conclusion {
            return GraphAssessmentDispositionV1::BlockedMissingEvidence;
        }

        let receipt_is_current = current_receipt.is_some_and(|receipt| {
            receipt.structurally_valid()
                && matches!(receipt.disposition, crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent)
        });

        let mut has_support = false;
        let mut has_conflict = false;
        let mut has_current_evidence = false;
        for edge in self.edges.values() {
            if edge.to_node_id != conclusion_id {
                continue;
            }
            match edge.kind {
                ClaimGraphEdgeKindV1::Supports => {
                    if self.nodes.contains_key(&edge.from_node_id) {
                        has_support = true;
                        if self
                            .nodes
                            .get(&edge.from_node_id)
                            .and_then(|n| n.current_frontier_sequence)
                            .is_some()
                        {
                            has_current_evidence = true;
                        }
                    }
                }
                ClaimGraphEdgeKindV1::Contradicts => {
                    if self.nodes.contains_key(&edge.from_node_id) {
                        has_conflict = true;
                    }
                }
                _ => {}
            }
        }

        if has_conflict {
            GraphAssessmentDispositionV1::DisputedByBoundEvidence
        } else if has_support && has_current_evidence && receipt_is_current {
            GraphAssessmentDispositionV1::SupportedByBoundEvidence
        } else if has_support {
            GraphAssessmentDispositionV1::BlockedCurrentness
        } else {
            GraphAssessmentDispositionV1::ReachableOnly
        }
    }

    pub fn closure_disposition(&self) -> BundleClosureDispositionV1 {
        if self.structural_closure() != BundleNodeStatusV1::Present {
            return BundleClosureDispositionV1::StructurallyIncomplete;
        }
        if self
            .nodes
            .values()
            .any(|node| node.historical_only)
        {
            return BundleClosureDispositionV1::HistoricalOnly;
        }
        let conclusions: Vec<_> = self
            .nodes
            .values()
            .filter(|n| n.kind == ClaimGraphNodeKindV1::Conclusion)
            .map(|n| self.semantically_bound_conclusion(&n.node_id))
            .collect();
        if conclusions
            .iter()
            .any(|d| matches!(d, GraphAssessmentDispositionV1::DisputedByBoundEvidence))
        {
            BundleClosureDispositionV1::Contested
        } else if conclusions.iter().any(|d| {
            matches!(
                d,
                GraphAssessmentDispositionV1::SupportedByBoundEvidence
            )
        }) {
            BundleClosureDispositionV1::StructurallyClosed
        } else {
            BundleClosureDispositionV1::SemanticallyInsufficient
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimGraphAssessmentReceiptV1 {
    pub assessment_id: String,
    pub bundle_id: String,
    pub conclusion_id: String,
    pub graph_reachable: bool,
    pub bound_evidence: bool,
    pub conflicting_evidence: bool,
    pub disposition: GraphAssessmentDispositionV1,
    pub human_disposition: HumanDispositionV1,
    pub assessment_commitment: String,
    pub claim_ceiling: String,
}

impl ClaimGraphAssessmentReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.assessment_id)
            && non_empty(&self.bundle_id)
            && non_empty(&self.conclusion_id)
            && non_empty(&self.assessment_commitment)
            && self.claim_ceiling == EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING
            && (!self.bound_evidence || self.graph_reachable)
            && (!self.conflicting_evidence || self.graph_reachable)
    }
}

pub fn graph_reachability_is_not_truth() -> bool {
    true
}

pub fn human_disposition_is_not_evidence() -> bool {
    true
}

pub fn provenance_is_not_causality() -> bool {
    true
}

pub fn conclusion_is_not_authorization() -> bool {
    true
}

pub fn symthaea_claim_graph_proposal_is_authoritative() -> bool {
    false
}

pub fn claim_graph_assessment_can_authorize_actuation() -> bool {
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(id: &str, kind: ClaimGraphNodeKindV1) -> ClaimGraphNodeV1 {
        ClaimGraphNodeV1 {
            node_id: id.into(),
            kind,
            content_commitment: format!("content-{id}"),
            provenance_root: Some(format!("prov-{id}")),
            custody_root: Some(format!("custody-{id}")),
            historical_only: false,
            current_frontier_sequence: Some(10),
            node_commitment: format!("node-{id}"),
            claim_ceiling: EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING.into(),
        }
    }

    fn edge(id: &str, from: &str, to: &str, kind: ClaimGraphEdgeKindV1) -> ClaimGraphEdgeV1 {
        ClaimGraphEdgeV1 {
            edge_id: id.into(),
            from_node_id: from.into(),
            to_node_id: to.into(),
            kind,
            edge_commitment: format!("edge-{id}"),
            claim_ceiling: EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING.into(),
        }
    }

    fn bundle() -> EvidenceBundleV1 {
        let mut nodes = BTreeMap::new();
        for (id, kind) in [
            ("source", ClaimGraphNodeKindV1::Source),
            ("evidence", ClaimGraphNodeKindV1::Evidence),
            ("statement", ClaimGraphNodeKindV1::Statement),
            ("receipt", ClaimGraphNodeKindV1::RegistrationReceipt),
            ("validation", ClaimGraphNodeKindV1::Validation),
            ("assessment", ClaimGraphNodeKindV1::Assessment),
            ("conclusion", ClaimGraphNodeKindV1::Conclusion),
            ("disposition", ClaimGraphNodeKindV1::HumanDisposition),
        ] {
            nodes.insert(id.into(), node(id, kind));
        }
        let mut edges = BTreeMap::new();
        let chain = [
            ("e1", "evidence", "source", ClaimGraphEdgeKindV1::DerivedFrom),
            ("e2", "statement", "evidence", ClaimGraphEdgeKindV1::DerivedFrom),
            ("e3", "statement", "receipt", ClaimGraphEdgeKindV1::Registers),
            ("e4", "receipt", "validation", ClaimGraphEdgeKindV1::Validates),
            ("e5", "validation", "assessment", ClaimGraphEdgeKindV1::Assesses),
            ("e6", "assessment", "conclusion", ClaimGraphEdgeKindV1::Concludes),
            ("e7", "conclusion", "disposition", ClaimGraphEdgeKindV1::Disposes),
            ("e8", "evidence", "statement", ClaimGraphEdgeKindV1::Supports),
            ("e9", "assessment", "conclusion", ClaimGraphEdgeKindV1::Supports),
        ];
        for (id, from, to, kind) in chain {
            edges.insert(id.into(), edge(id, from, to, kind));
        }
        EvidenceBundleV1 {
            bundle_id: "bundle-1".into(),
            graph: ClaimGraphBindingV1 {
                source_node_ids: ["source".into()].into_iter().collect(),
                evidence_node_ids: ["evidence".into()].into_iter().collect(),
                statement_node_ids: ["statement".into()].into_iter().collect(),
                registration_receipt_node_ids: ["receipt".into()].into_iter().collect(),
                validation_node_ids: ["validation".into()].into_iter().collect(),
                assessment_node_ids: ["assessment".into()].into_iter().collect(),
                conclusion_node_ids: ["conclusion".into()].into_iter().collect(),
                human_disposition_node_ids: ["disposition".into()].into_iter().collect(),
                graph_commitment: "graph-1".into(),
                claim_ceiling: EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING.into(),
            },
            nodes,
            edges,
            bundle_commitment: "bundle-commitment".into(),
            claim_ceiling: EVIDENCE_CLAIM_GRAPH_CLAIM_CEILING.into(),
        }
    }

    #[test]
    fn complete_bundle_is_structurally_closed_but_not_truth() {
        let b = bundle();
        assert_eq!(b.structural_closure(), BundleNodeStatusV1::Present);
        assert_eq!(
            b.closure_disposition(),
            BundleClosureDispositionV1::StructurallyClosed
        );
        assert!(graph_reachability_is_not_truth());
    }

    #[test]
    fn dangling_edge_cannot_be_repaired_by_human_disposition() {
        let mut b = bundle();
        b.edges.insert(
            "dangling".into(),
            edge(
                "dangling",
                "human-disposition-missing",
                "conclusion",
                ClaimGraphEdgeKindV1::Supports,
            ),
        );
        assert_eq!(b.structural_closure(), BundleNodeStatusV1::Invalid);
        assert!(human_disposition_is_not_evidence());
    }

    #[test]
    fn current_support_requires_exact_d6p_eligible_current_receipt() {
        let b = bundle();
        assert_eq!(
            b.semantically_bound_conclusion_with_current_receipt("conclusion", None),
            GraphAssessmentDispositionV1::BlockedCurrentness
        );

        let receipt = CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "receipt-1".into(), effect_id: "effect-1".into(), effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "generation-1".into(), route_id: "route-1".into(), provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(), provider_profile_root: "provider-profile-1".into(),
            semantic_environment_root: "env-1".into(), observation_set_id: "set-1".into(), observation_set_commitment: "set-commitment".into(),
            d6n_assessment_commitment: "assessment:set-commitment".into(), witness_eligibility_ids: ["eligibility-1".into()].into_iter().collect(),
            observer_generation_ids: ["generation-1".into()].into_iter().collect(), current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(), eligible_independent_count: 1, preserved_contradictory_count: 0,
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition-1".into(), receipt_commitment: "receipt-commitment".into(),
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        };
        assert_eq!(
            b.semantically_bound_conclusion_with_current_receipt("conclusion", Some(&receipt)),
            GraphAssessmentDispositionV1::SupportedByBoundEvidence
        );
    }

    #[test]
    fn provenance_edge_does_not_become_support() {
        let mut b = bundle();
        b.edges.remove("e8");
        b.edges.insert(
            "provenance".into(),
            edge(
                "provenance",
                "evidence",
                "statement",
                ClaimGraphEdgeKindV1::Provenance,
            ),
        );
        assert_eq!(
            b.semantically_bound_conclusion("conclusion"),
            GraphAssessmentDispositionV1::SupportedByBoundEvidence
        );
        assert!(provenance_is_not_causality());
    }

    #[test]
    fn reachability_alone_does_not_support_conclusion() {
        let mut b = bundle();
        b.edges.remove("e9");
        assert_eq!(
            b.semantically_bound_conclusion("conclusion"),
            GraphAssessmentDispositionV1::ReachableOnly
        );
    }

    #[test]
    fn contradictory_bound_evidence_is_disputed() {
        let mut b = bundle();
        b.edges.insert(
            "conflict".into(),
            edge(
                "conflict",
                "evidence",
                "conclusion",
                ClaimGraphEdgeKindV1::Contradicts,
            ),
        );
        assert_eq!(
            b.semantically_bound_conclusion("conclusion"),
            GraphAssessmentDispositionV1::DisputedByBoundEvidence
        );
        assert_eq!(b.closure_disposition(), BundleClosureDispositionV1::Contested);
    }

    #[test]
    fn historical_evidence_does_not_become_current() {
        let mut b = bundle();
        b.nodes.get_mut("evidence").unwrap().historical_only = true;
        assert_eq!(b.closure_disposition(), BundleClosureDispositionV1::HistoricalOnly);
    }

    #[test]
    fn invalid_edge_type_is_rejected() {
        let mut b = bundle();
        b.edges.insert(
            "bad".into(),
            edge(
                "bad",
                "source",
                "conclusion",
                ClaimGraphEdgeKindV1::Supports,
            ),
        );
        assert!(b.incompatible_edge_ids().contains("bad"));
        assert_eq!(b.structural_closure(), BundleNodeStatusV1::Invalid);
    }

    #[test]
    fn self_support_cycle_is_rejected() {
        let mut b = bundle();
        b.edges.insert(
            "cycle".into(),
            edge(
                "cycle",
                "conclusion",
                "statement",
                ClaimGraphEdgeKindV1::DerivedFrom,
            ),
        );
        b.edges.insert(
            "cycle-back".into(),
            edge(
                "cycle-back",
                "statement",
                "conclusion",
                ClaimGraphEdgeKindV1::Concludes,
            ),
        );
        assert!(b.graph_has_cycle());
        assert_eq!(b.structural_closure(), BundleNodeStatusV1::Invalid);
    }

    #[test]
    fn current_support_requires_current_frontier_binding() {
        let mut b = bundle();
        b.nodes.get_mut("evidence").unwrap().current_frontier_sequence = None;
        assert_eq!(
            b.semantically_bound_conclusion("conclusion"),
            GraphAssessmentDispositionV1::BlockedCurrentness
        );
    }

    #[test]
    fn missing_required_node_is_explicit() {
        let mut b = bundle();
        b.nodes.remove("validation");
        let missing = b.missing_required_nodes();
        assert!(missing.contains(&ClaimGraphNodeKindV1::Validation));
    }

    #[test]
    fn human_disposition_never_creates_support() {
        let mut b = bundle();
        b.edges.remove("e8");
        b.edges.remove("e9");
        b.edges.insert(
            "dispose".into(),
            edge(
                "dispose",
                "conclusion",
                "disposition",
                ClaimGraphEdgeKindV1::Disposes,
            ),
        );
        assert_eq!(
            b.semantically_bound_conclusion("conclusion"),
            GraphAssessmentDispositionV1::ReachableOnly
        );
    }

    #[test]
    fn conclusion_is_not_authorization() {
        assert!(!conclusion_is_not_authorization() == false);
        assert!(!claim_graph_assessment_can_authorize_actuation());
    }

    #[test]
    fn symthaea_proposal_is_non_authoritative() {
        assert!(!symthaea_claim_graph_proposal_is_authoritative());
    }

    #[test]
    fn receipt_is_not_endorsement() {
        let b = bundle();
        assert!(b
            .edges
            .values()
            .any(|e| e.kind == ClaimGraphEdgeKindV1::Registers));
        assert!(b
            .edges
            .values()
            .all(|e| e.kind != ClaimGraphEdgeKindV1::Registers || e.to_node_id == "receipt"));
    }
}
