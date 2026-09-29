//! D6L evidence bundles and claim graphs.
//!
//! This module makes evidence/claim relationships explicit without collapsing
//! source, evidence, registration, validation, conclusion, or disposition.
//! Graph connectivity is descriptive: it never manufactures a conclusion,
//! authority, truth claim, or causal edge.
//!
//! Claim ceiling: ReferenceModelOnly.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClaimNodeKind {
    Source,
    Evidence,
    Statement,
    Receipt,
    Validation,
    Assessment,
    Conclusion,
    HumanDisposition,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClaimRelation {
    Provides,
    Supports,
    Disputes,
    DerivedFrom,
    RegisteredAs,
    ValidatedAs,
    AssessedAs,
    Concludes,
    DispositionOf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClaimOrigin {
    Local,
    Foreign,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClaimNode {
    pub node_id: &'static str,
    pub kind: ClaimNodeKind,
    pub origin: ClaimOrigin,
    pub generation: u32,
    pub source_ref: Option<&'static str>,
    pub evidence_ref: Option<&'static str>,
    pub digest: Option<&'static str>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClaimEdge {
    pub from: &'static str,
    pub to: &'static str,
    pub relation: ClaimRelation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvidenceBundle {
    pub bundle_id: &'static str,
    pub generation: u32,
    pub items: Vec<ClaimNode>,
    pub edges: Vec<ClaimEdge>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClaimGraphError {
    EmptyIdentity,
    SelfRelation,
    MissingEndpoint,
    DuplicateNode,
    DuplicateEdge,
    MissingEvidenceBinding,
    MissingSourceBinding,
    ForeignOriginMismatch,
    GenerationMismatch,
    InvalidRelation,
    ConclusionDisconnected,
    DispositionDisconnected,
}

impl EvidenceBundle {
    pub fn validate(&self) -> Result<(), ClaimGraphError> {
        if self.bundle_id.trim().is_empty() {
            return Err(ClaimGraphError::EmptyIdentity);
        }

        for node in &self.items {
            if node.node_id.trim().is_empty() {
                return Err(ClaimGraphError::EmptyIdentity);
            }
            if node.generation != self.generation {
                return Err(ClaimGraphError::GenerationMismatch);
            }
            if matches!(node.kind, ClaimNodeKind::Evidence | ClaimNodeKind::Statement | ClaimNodeKind::Receipt)
                && node.evidence_ref.map_or(true, str::is_empty)
            {
                return Err(ClaimGraphError::MissingEvidenceBinding);
            }
            if matches!(node.kind, ClaimNodeKind::Source | ClaimNodeKind::Evidence | ClaimNodeKind::Statement)
                && node.source_ref.map_or(true, str::is_empty)
            {
                return Err(ClaimGraphError::MissingSourceBinding);
            }
        }

        for (i, left) in self.items.iter().enumerate() {
            if self.items[..i].iter().any(|right| right.node_id == left.node_id) {
                return Err(ClaimGraphError::DuplicateNode);
            }
        }

        for edge in &self.edges {
            if edge.from == edge.to {
                return Err(ClaimGraphError::SelfRelation);
            }
            let from = self.items.iter().find(|n| n.node_id == edge.from);
            let to = self.items.iter().find(|n| n.node_id == edge.to);
            let (Some(from), Some(to)) = (from, to) else {
                return Err(ClaimGraphError::MissingEndpoint);
            };

            if edge.relation == ClaimRelation::Supports
                || edge.relation == ClaimRelation::Disputes
            {
                if !matches!(from.kind, ClaimNodeKind::Evidence | ClaimNodeKind::Statement | ClaimNodeKind::Assessment)
                    || !matches!(to.kind, ClaimNodeKind::Statement | ClaimNodeKind::Assessment | ClaimNodeKind::Conclusion)
                {
                    return Err(ClaimGraphError::InvalidRelation);
                }
            }

            if edge.relation == ClaimRelation::RegisteredAs
                && !(from.kind == ClaimNodeKind::Statement && to.kind == ClaimNodeKind::Receipt)
            {
                return Err(ClaimGraphError::InvalidRelation);
            }

            if edge.relation == ClaimRelation::ValidatedAs
                && !(from.kind == ClaimNodeKind::Statement && to.kind == ClaimNodeKind::Validation)
            {
                return Err(ClaimGraphError::InvalidRelation);
            }

            if edge.relation == ClaimRelation::Concludes
                && !(from.kind == ClaimNodeKind::Assessment && to.kind == ClaimNodeKind::Conclusion)
            {
                return Err(ClaimGraphError::InvalidRelation);
            }

            if edge.relation == ClaimRelation::DispositionOf
                && !(from.kind == ClaimNodeKind::HumanDisposition && to.kind == ClaimNodeKind::Conclusion)
            {
                return Err(ClaimGraphError::InvalidRelation);
            }

            if matches!(edge.relation, ClaimRelation::Provides | ClaimRelation::DerivedFrom)
                && from.origin != to.origin
                && !matches!(edge.relation, ClaimRelation::DerivedFrom)
            {
                return Err(ClaimGraphError::ForeignOriginMismatch);
            }
        }

        for (i, left) in self.edges.iter().enumerate() {
            if self.edges[..i].iter().any(|right| right == left) {
                return Err(ClaimGraphError::DuplicateEdge);
            }
        }

        // Conclusions must be connected to explicit evidence/assessment paths.
        for conclusion in self.items.iter().filter(|n| n.kind == ClaimNodeKind::Conclusion) {
            let connected = self.edges.iter().any(|e| {
                e.to == conclusion.node_id
                    && matches!(e.relation, ClaimRelation::Supports | ClaimRelation::Disputes | ClaimRelation::Concludes)
            });
            if !connected {
                return Err(ClaimGraphError::ConclusionDisconnected);
            }
        }

        // A disposition records governance state; it does not become evidence.
        for disposition in self.items.iter().filter(|n| n.kind == ClaimNodeKind::HumanDisposition) {
            if !self.edges.iter().any(|e| {
                e.from == disposition.node_id && e.relation == ClaimRelation::DispositionOf
            }) {
                return Err(ClaimGraphError::DispositionDisconnected);
            }
        }

        Ok(())
    }
}

/// Semantic equivalence ignores node/edge serialization order, but preserves
/// every semantic field and rejects duplicate identities through validation.
pub fn semantically_equivalent(left: &EvidenceBundle, right: &EvidenceBundle) -> bool {
    if left.validate().is_err() || right.validate().is_err() {
        return false;
    }
    if left.bundle_id != right.bundle_id || left.generation != right.generation {
        return false;
    }

    let mut left_nodes = left.items.clone();
    let mut right_nodes = right.items.clone();
    left_nodes.sort_by_key(|n| n.node_id);
    right_nodes.sort_by_key(|n| n.node_id);

    let mut left_edges = left.edges.clone();
    let mut right_edges = right.edges.clone();
    left_edges.sort_by_key(|e| (e.from, e.to, e.relation as u8));
    right_edges.sort_by_key(|e| (e.from, e.to, e.relation as u8));

    left_nodes == right_nodes && left_edges == right_edges
}

/// Explicit bridge for D6J/D6K: these identities may be connected, but no
/// conclusion is inferred merely because registration and validation exist.
pub fn receipt_validation_bridge(
    receipt: &ClaimNode,
    validation: &ClaimNode,
) -> bool {
    receipt.kind == ClaimNodeKind::Receipt
        && validation.kind == ClaimNodeKind::Validation
        && receipt.generation == validation.generation
        && receipt.evidence_ref.is_some()
        && validation.evidence_ref.is_some()
        && receipt.evidence_ref == validation.evidence_ref
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(
        id: &'static str,
        kind: ClaimNodeKind,
        source: Option<&'static str>,
        evidence: Option<&'static str>,
    ) -> ClaimNode {
        ClaimNode {
            node_id: id,
            kind,
            origin: ClaimOrigin::Foreign,
            generation: 7,
            source_ref: source,
            evidence_ref: evidence,
            digest: Some("digest:opaque"),
        }
    }

    fn bundle() -> EvidenceBundle {
        EvidenceBundle {
            bundle_id: "bundle-1",
            generation: 7,
            items: vec![
                node("source-1", ClaimNodeKind::Source, Some("source://n2"), None),
                node("evidence-1", ClaimNodeKind::Evidence, Some("source://n2"), Some("evidence://n2/1")),
                node("statement-1", ClaimNodeKind::Statement, Some("source://n2"), Some("evidence://n2/1")),
                node("receipt-1", ClaimNodeKind::Receipt, Some("source://n2"), Some("evidence://n2/1")),
                node("validation-1", ClaimNodeKind::Validation, Some("source://n2"), Some("evidence://n2/1")),
                node("assessment-1", ClaimNodeKind::Assessment, Some("source://n2"), None),
                node("conclusion-1", ClaimNodeKind::Conclusion, Some("source://n2"), None),
                node("disposition-1", ClaimNodeKind::HumanDisposition, Some("source://n2"), None),
            ],
            edges: vec![
                ClaimEdge { from: "source-1", to: "evidence-1", relation: ClaimRelation::Provides },
                ClaimEdge { from: "evidence-1", to: "statement-1", relation: ClaimRelation::Supports },
                ClaimEdge { from: "statement-1", to: "receipt-1", relation: ClaimRelation::RegisteredAs },
                ClaimEdge { from: "statement-1", to: "validation-1", relation: ClaimRelation::ValidatedAs },
                ClaimEdge { from: "assessment-1", to: "conclusion-1", relation: ClaimRelation::Concludes },
                ClaimEdge { from: "disposition-1", to: "conclusion-1", relation: ClaimRelation::DispositionOf },
            ],
        }
    }

    #[test]
    fn bundle_preserves_distinct_source_and_evidence_bindings() {
        assert!(bundle().validate().is_ok());
        let statement = bundle().items.into_iter().find(|n| n.node_id == "statement-1").unwrap();
        assert_ne!(statement.source_ref, statement.evidence_ref);
    }

    #[test]
    fn conclusion_requires_explicit_graph_support_or_assessment_path() {
        let mut b = bundle();
        b.edges.retain(|e| e.from != "assessment-1");
        assert_eq!(b.validate(), Err(ClaimGraphError::ConclusionDisconnected));
    }

    #[test]
    fn disposition_is_not_evidence() {
        let b = bundle();
        let disposition = b.items.iter().find(|n| n.kind == ClaimNodeKind::HumanDisposition).unwrap();
        assert!(disposition.evidence_ref.is_none());
        assert!(b.validate().is_ok());
    }

    #[test]
    fn receipt_and_validation_require_explicit_same_evidence_binding() {
        let b = bundle();
        let receipt = b.items.iter().find(|n| n.kind == ClaimNodeKind::Receipt).unwrap();
        let validation = b.items.iter().find(|n| n.kind == ClaimNodeKind::Validation).unwrap();
        assert!(receipt_validation_bridge(receipt, validation));

        let mut changed = *validation;
        changed.evidence_ref = Some("evidence://other/9");
        assert!(!receipt_validation_bridge(receipt, &changed));
    }

    #[test]
    fn serialization_order_does_not_change_semantics() {
        let left = bundle();
        let mut right = bundle();
        right.items.reverse();
        right.edges.reverse();
        assert!(semantically_equivalent(&left, &right));
    }

    #[test]
    fn source_or_evidence_mutation_breaks_equivalence() {
        let left = bundle();
        let mut right = bundle();
        right.items[2].evidence_ref = Some("evidence://other/9");
        assert!(!semantically_equivalent(&left, &right));

        let mut right = bundle();
        right.items[2].source_ref = Some("source://other");
        assert!(!semantically_equivalent(&left, &right));
    }

    #[test]
    fn foreign_origin_cannot_be_laundered_to_local() {
        let left = bundle();
        let mut right = bundle();
        right.items[1].origin = ClaimOrigin::Local;
        assert!(!semantically_equivalent(&left, &right));
    }

    #[test]
    fn duplicate_nodes_and_edges_fail_closed() {
        let mut b = bundle();
        b.items.push(b.items[0]);
        assert_eq!(b.validate(), Err(ClaimGraphError::DuplicateNode));

        let mut b = bundle();
        b.edges.push(b.edges[0]);
        assert_eq!(b.validate(), Err(ClaimGraphError::DuplicateEdge));
    }

    #[test]
    fn graph_connectivity_does_not_create_a_conclusion() {
        let mut b = bundle();
        b.items.retain(|n| n.kind != ClaimNodeKind::Conclusion);
        assert!(b.validate().is_ok());
        // No conclusion node means no conclusion is inferred from connectivity.
        assert!(!b.items.iter().any(|n| n.kind == ClaimNodeKind::Conclusion));
    }
}
