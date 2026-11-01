//! Bitemporal warning-evidence graph and CAP lifecycle conformance core.
//!
//! This is an offline synthetic audit benchmark. It does not issue alerts,
//! establish legal warning authority, or certify physical truth.
//!
//! Important boundary: this crate consumes a DigestStatus supplied by an
//! external cryptographic verifier. It does not implement or claim to verify
//! SHA-256 signatures/digests itself. Fixture checksums are not security
//! primitives.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum NodeKind {
    SourceObservation,
    ForecastProduct,
    AuthorityAssertion,
    CapMessage,
    DeliveryReceipt,
    AccessibleRendering,
    ComprehensionEvidence,
    ActionEvidence,
    OutcomeEvaluation,
    Activity,
    Agent,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DigestStatus {
    Unchecked,
    VerifiedByExternalAdapter,
    /// Accepted only by the frozen synthetic fixture oracle; not a cryptographic result.
    FixtureOracleAccepted,
    Mismatch,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvidenceNode {
    pub id: String,
    pub kind: NodeKind,
    /// Event/claim-time applicability, not transaction time.
    pub valid_from: Option<u64>,
    pub valid_to: Option<u64>,
    /// Transaction/knowledge time at which the graph recorded this node.
    pub recorded_at: u64,
    /// Source-declared creation time; never substituted for recorded_at.
    pub source_created_at: Option<u64>,
    pub payload_digest: Option<String>,
    pub digest_status: DigestStatus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum RelationKind {
    DerivedFrom,
    Supports,
    Contradicts,
    Supersedes,
    Revokes,
    References,
    DeliveredAs,
    RenderedAs,
    AssessedBy,
    ActionEvidenceFor,
    EvaluatedBy,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvidenceEdge {
    pub id: String,
    pub from: String,
    pub to: String,
    pub kind: RelationKind,
    /// Valid/effective time of the relation, where meaningful.
    pub valid_at: Option<u64>,
    /// Transaction time when the relation became known.
    pub recorded_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeConflict {
    pub original: EvidenceNode,
    pub conflicting: EvidenceNode,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EvidenceGraph {
    pub nodes: BTreeMap<String, EvidenceNode>,
    pub edges: Vec<EvidenceEdge>,
    pub node_conflicts: Vec<NodeConflict>,
    pub exact_duplicate_nodes: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GraphSnapshot {
    pub as_of_transaction_time: u64,
    pub nodes: BTreeMap<String, EvidenceNode>,
    /// Keep edges even if an endpoint had not yet arrived at this cutoff:
    /// validate() then reports the historically dangling reference.
    pub edges: Vec<EvidenceEdge>,
    pub node_conflicts: Vec<NodeConflict>,
}

impl EvidenceGraph {
    /// Append a node. Exact duplicate identity+content is idempotent; a
    /// conflicting duplicate is retained in node_conflicts and rejected.
    pub fn append_node(&mut self, node: EvidenceNode) -> Result<bool, String> {
        match self.nodes.get(&node.id) {
            None => {
                self.nodes.insert(node.id.clone(), node);
                Ok(true)
            }
            Some(existing) if existing == &node => {
                self.exact_duplicate_nodes += 1;
                Ok(false)
            }
            Some(existing) => {
                self.node_conflicts.push(NodeConflict {
                    original: existing.clone(),
                    conflicting: node,
                });
                Err("conflicting-node-identity".into())
            }
        }
    }

    pub fn append_edge(&mut self, edge: EvidenceEdge) {
        self.edges.push(edge);
    }

    /// Reconstruct what had been recorded by transaction-time cutoff T.
    /// valid_from/valid_to/valid_at remain attached to the returned records.
    pub fn snapshot_as_of(&self, cutoff: u64) -> GraphSnapshot {
        let nodes: BTreeMap<String, EvidenceNode> = self
            .nodes
            .iter()
            .filter(|(_, node)| node.recorded_at <= cutoff)
            .map(|(id, node)| (id.clone(), node.clone()))
            .collect();
        GraphSnapshot {
            as_of_transaction_time: cutoff,
            nodes,
            edges: self
                .edges
                .iter()
                .filter(|edge| edge.recorded_at <= cutoff)
                .cloned()
                .collect(),
            node_conflicts: self
                .node_conflicts
                .iter()
                .filter(|conflict| conflict.conflicting.recorded_at <= cutoff)
                .cloned()
                .collect(),
        }
    }

    pub fn validate(&self) -> GraphReport {
        validate_parts(
            &self.nodes,
            &self.edges,
            self.exact_duplicate_nodes,
            self.node_conflicts.len() as u32,
        )
    }
}

impl GraphSnapshot {
    pub fn validate(&self) -> GraphReport {
        validate_parts(
            &self.nodes,
            &self.edges,
            0,
            self.node_conflicts.len() as u32,
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GraphReport {
    pub node_count: u32,
    pub edge_count: u32,
    pub exact_duplicate_nodes: u32,
    pub conflicting_node_identities: u32,
    pub conflicting_edge_identities: u32,
    pub dangling_edges: u32,
    pub invalid_relation_types: u32,
    /// Relations recorded before one or more referenced endpoint records arrived.
    /// Informational: historical snapshots show these as dangling until resolution.
    pub forward_reference_edges: u32,
    pub invalid_validity_intervals: u32,
    pub unverified_payload_digests: u32,
    pub digest_mismatches: u32,
    pub forbidden_cycles: u32,
}

impl GraphReport {
    pub fn is_conformant(&self) -> bool {
        self.conflicting_node_identities == 0
            && self.conflicting_edge_identities == 0
            && self.dangling_edges == 0
            && self.invalid_relation_types == 0
            && self.invalid_validity_intervals == 0
            && self.unverified_payload_digests == 0
            && self.digest_mismatches == 0
            && self.forbidden_cycles == 0
    }
}

fn relation_endpoints_valid(kind: RelationKind, from: NodeKind, to: NodeKind) -> bool {
    use NodeKind::*;
    use RelationKind::*;
    match kind {
        DerivedFrom => matches!(
            (from, to),
            (ForecastProduct, SourceObservation)
                | (CapMessage, ForecastProduct)
                | (CapMessage, AuthorityAssertion)
                | (AccessibleRendering, CapMessage)
                | (ComprehensionEvidence, AccessibleRendering)
                | (ActionEvidence, CapMessage)
                | (OutcomeEvaluation, ActionEvidence)
        ),
        Supports => matches!((from, to), (AuthorityAssertion, ForecastProduct)),
        Contradicts => matches!(
            (from, to),
            (SourceObservation, SourceObservation)
                | (SourceObservation, ForecastProduct)
                | (ForecastProduct, SourceObservation)
        ),
        Supersedes => from == to,
        Revokes => matches!((from, to), (AuthorityAssertion, AuthorityAssertion)),
        References => matches!((from, to), (CapMessage, CapMessage)),
        DeliveredAs => matches!((from, to), (DeliveryReceipt, CapMessage)),
        RenderedAs => matches!((from, to), (AccessibleRendering, CapMessage)),
        AssessedBy => matches!((from, to), (ComprehensionEvidence, AccessibleRendering)),
        ActionEvidenceFor => matches!((from, to), (ActionEvidence, CapMessage)),
        EvaluatedBy => matches!((from, to), (OutcomeEvaluation, ActionEvidence)),
    }
}

fn has_forbidden_cycle(nodes: &BTreeMap<String, EvidenceNode>, edges: &[EvidenceEdge]) -> bool {
    let mut adjacency: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for edge in edges {
        if matches!(
            edge.kind,
            RelationKind::DerivedFrom | RelationKind::Supersedes | RelationKind::Revokes
        ) && nodes.contains_key(&edge.from)
            && nodes.contains_key(&edge.to)
        {
            adjacency
                .entry(edge.from.clone())
                .or_default()
                .push(edge.to.clone());
        }
    }

    fn visit(
        node: &str,
        adjacency: &BTreeMap<String, Vec<String>>,
        state: &mut BTreeMap<String, u8>,
    ) -> bool {
        match state.get(node).copied().unwrap_or(0) {
            1 => return true,
            2 => return false,
            _ => {}
        }
        state.insert(node.to_string(), 1);
        if let Some(children) = adjacency.get(node) {
            for child in children {
                if visit(child, adjacency, state) {
                    return true;
                }
            }
        }
        state.insert(node.to_string(), 2);
        false
    }

    let mut state: BTreeMap<String, u8> = BTreeMap::new();
    for node in nodes.keys() {
        if visit(node, &adjacency, &mut state) {
            return true;
        }
    }
    false
}

fn validate_parts(
    nodes: &BTreeMap<String, EvidenceNode>,
    edges: &[EvidenceEdge],
    exact_duplicates: u32,
    conflicting_nodes: u32,
) -> GraphReport {
    let mut report = GraphReport {
        node_count: nodes.len() as u32,
        edge_count: edges.len() as u32,
        exact_duplicate_nodes: exact_duplicates,
        conflicting_node_identities: conflicting_nodes,
        ..GraphReport::default()
    };
    for node in nodes.values() {
        if matches!((node.valid_from, node.valid_to), (Some(start), Some(end)) if end < start) {
            report.invalid_validity_intervals += 1;
        }
        if node.payload_digest.is_some() {
            match node.digest_status {
                DigestStatus::Unchecked => report.unverified_payload_digests += 1,
                DigestStatus::Mismatch => report.digest_mismatches += 1,
                DigestStatus::VerifiedByExternalAdapter | DigestStatus::FixtureOracleAccepted => {}
            }
        }
    }

    let mut edge_ids = BTreeSet::new();
    for edge in edges {
        if !edge_ids.insert(edge.id.as_str()) {
            report.conflicting_edge_identities += 1;
        }
        let from = nodes.get(&edge.from);
        let to = nodes.get(&edge.to);
        match (from, to) {
            (Some(from_node), Some(to_node)) => {
                if !relation_endpoints_valid(edge.kind, from_node.kind, to_node.kind) {
                    report.invalid_relation_types += 1;
                }
                if edge.recorded_at < from_node.recorded_at
                    || edge.recorded_at < to_node.recorded_at
                {
                    // Distributed evidence may reference an entity before its
                    // delayed payload arrives. Preserve this as an observable
                    // forward reference rather than declaring the current graph
                    // invalid; an earlier as-of snapshot will show it dangling.
                    report.forward_reference_edges += 1;
                }
            }
            _ => report.dangling_edges += 1,
        }
    }

    if has_forbidden_cycle(nodes, edges) {
        report.forbidden_cycles = 1;
    }
    report
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapMessageType {
    Alert,
    Update,
    Cancel,
    Ack,
    Error,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DigestVerification {
    Unchecked,
    VerifiedByExternalAdapter,
    /// Fixture-only trust marker; never treat this as a production digest check.
    FixtureOracleAccepted,
    Mismatch,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapMessage {
    pub sender: String,
    pub identifier: String,
    /// Monotonic synthetic seconds in this fixture. Production adapters must
    /// parse CAP dateTime and preserve its actual timezone/precision.
    pub sent_at: u64,
    pub message_type: CapMessageType,
    /// Exact CAP extended message identifiers: sender,identifier,sent.
    pub references: Vec<String>,
    pub expires_at: Option<u64>,
    pub recorded_at: u64,
    pub wire_bytes: Vec<u8>,
    pub declared_digest: String,
    /// Result supplied by an external cryptographic digest/signature adapter.
    pub digest_verification: DigestVerification,
}

impl CapMessage {
    pub fn extended_identifier(&self) -> String {
        format!("{},{},{}", self.sender, self.identifier, self.sent_at)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CapLifecycleReport {
    pub messages_seen: u32,
    pub exact_duplicates: u32,
    pub conflicting_identifiers: u32,
    pub updates: u32,
    pub cancellations: u32,
    pub missing_references: u32,
    pub unknown_references: u32,
    pub non_prior_references: u32,
    pub expired_references: u32,
    pub unverified_digests: u32,
    pub digest_mismatches: u32,
}

impl CapLifecycleReport {
    /// Core structural conformance. CAP 1.2 requires earlier-message
    /// references for Update/Cancel; expiry policy is a separate profile rule.
    pub fn is_conformant(&self) -> bool {
        self.conflicting_identifiers == 0
            && self.missing_references == 0
            && self.unknown_references == 0
            && self.non_prior_references == 0
            && self.unverified_digests == 0
            && self.digest_mismatches == 0
    }

    /// Optional stricter profile: all referenced messages must still be active.
    /// This is not asserted as a universal CAP 1.2 core requirement.
    pub fn is_conformant_under_active_reference_profile(&self) -> bool {
        self.is_conformant() && self.expired_references == 0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapLifecycleAudit {
    pub report: CapLifecycleReport,
    /// Preserve conflicting payloads for independent review; do not silently
    /// select one record for a reused CAP extended identifier.
    pub conflicting_messages: Vec<CapMessage>,
}

/// Validate an append-ordered synthetic CAP message history.
///
/// Update and Cancel must resolve exact earlier extended identifiers. The
/// function never edits predecessor message bytes. CAP XML parsing and actual
/// cryptographic verification are separate adapters and are not implemented
/// by this core.
pub fn validate_cap_lifecycle(messages: &[CapMessage]) -> CapLifecycleAudit {
    let mut report = CapLifecycleReport {
        messages_seen: messages.len() as u32,
        ..CapLifecycleReport::default()
    };
    let mut conflicting_messages = Vec::new();
    let mut seen: BTreeMap<String, &CapMessage> = BTreeMap::new();

    for message in messages {
        match message.digest_verification {
            DigestVerification::Unchecked => report.unverified_digests += 1,
            DigestVerification::Mismatch => report.digest_mismatches += 1,
            DigestVerification::VerifiedByExternalAdapter | DigestVerification::FixtureOracleAccepted => {}
        }

        let extended_id = message.extended_identifier();
        if let Some(previous) = seen.get(&extended_id) {
            if previous.wire_bytes == message.wire_bytes
                && previous.declared_digest == message.declared_digest
            {
                report.exact_duplicates += 1;
            } else {
                report.conflicting_identifiers += 1;
                conflicting_messages.push(message.clone());
            }
            continue;
        }

        if matches!(message.message_type, CapMessageType::Update | CapMessageType::Cancel) {
            if message.references.is_empty() {
                report.missing_references += 1;
            } else {
                for reference in &message.references {
                    match seen.get(reference) {
                        None => report.unknown_references += 1,
                        Some(previous) => {
                            if previous.sent_at >= message.sent_at {
                                report.non_prior_references += 1;
                            }
                            if previous
                                .expires_at
                                .is_some_and(|expires| expires <= message.sent_at)
                            {
                                report.expired_references += 1;
                            }
                        }
                    }
                }
            }
            match message.message_type {
                CapMessageType::Update => report.updates += 1,
                CapMessageType::Cancel => report.cancellations += 1,
                _ => {}
            }
        }
        seen.insert(extended_id, message);
    }
    CapLifecycleAudit { report, conflicting_messages }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeliveryReceipt {
    pub receipt_id: String,
    pub message_reference: String,
    pub message_digest: String,
    pub recorded_at: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ReceiptReport {
    pub receipts_seen: u32,
    pub exact_message_matches: u32,
    pub unknown_messages: u32,
    pub conflicting_message_identities: u32,
    pub receipts_for_ambiguous_messages: u32,
    pub digest_mismatches: u32,
    pub unverified_message_digests: u32,
}

impl ReceiptReport {
    pub fn is_conformant(&self) -> bool {
        self.unknown_messages == 0
            && self.conflicting_message_identities == 0
            && self.receipts_for_ambiguous_messages == 0
            && self.digest_mismatches == 0
            && self.unverified_message_digests == 0
    }
}

pub fn validate_receipts(messages: &[CapMessage], receipts: &[DeliveryReceipt]) -> ReceiptReport {
    let mut by_id: BTreeMap<String, &CapMessage> = BTreeMap::new();
    let mut ambiguous_ids: BTreeSet<String> = BTreeSet::new();
    for message in messages {
        let id = message.extended_identifier();
        match by_id.get(&id) {
            None => {
                by_id.insert(id, message);
            }
            Some(previous)
                if previous.wire_bytes == message.wire_bytes
                    && previous.declared_digest == message.declared_digest => {}
            Some(_) => {
                ambiguous_ids.insert(id);
            }
        }
    }

    let mut report = ReceiptReport {
        receipts_seen: receipts.len() as u32,
        conflicting_message_identities: ambiguous_ids.len() as u32,
        ..ReceiptReport::default()
    };

    for receipt in receipts {
        if ambiguous_ids.contains(&receipt.message_reference) {
            report.receipts_for_ambiguous_messages += 1;
            continue;
        }
        match by_id.get(&receipt.message_reference) {
            None => report.unknown_messages += 1,
            Some(message) => {
                if receipt.message_digest != message.declared_digest {
                    report.digest_mismatches += 1;
                } else if !matches!(
                    message.digest_verification,
                    DigestVerification::VerifiedByExternalAdapter | DigestVerification::FixtureOracleAccepted
                ) {
                    report.unverified_message_digests += 1;
                } else {
                    report.exact_message_matches += 1;
                }
            }
        }
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(
        id: &str,
        kind: NodeKind,
        valid_from: Option<u64>,
        valid_to: Option<u64>,
        recorded_at: u64,
    ) -> EvidenceNode {
        EvidenceNode {
            id: id.into(),
            kind,
            valid_from,
            valid_to,
            recorded_at,
            source_created_at: valid_from,
            payload_digest: None,
            digest_status: DigestStatus::Unchecked,
        }
    }

    fn edge(
        id: &str,
        from: &str,
        to: &str,
        kind: RelationKind,
        valid_at: Option<u64>,
        recorded_at: u64,
    ) -> EvidenceEdge {
        EvidenceEdge {
            id: id.into(),
            from: from.into(),
            to: to.into(),
            kind,
            valid_at,
            recorded_at,
        }
    }

    fn cap_message(
        identifier: &str,
        sent_at: u64,
        message_type: CapMessageType,
        references: Vec<String>,
        expires_at: Option<u64>,
        bytes: &[u8],
    ) -> CapMessage {
        CapMessage {
            sender: "authority.example".into(),
            identifier: identifier.into(),
            sent_at,
            message_type,
            references,
            expires_at,
            recorded_at: sent_at,
            wire_bytes: bytes.to_vec(),
            declared_digest: format!("sha256-fixture-{identifier}"),
            digest_verification: DigestVerification::FixtureOracleAccepted,
        }
    }

    #[test]
    fn historical_as_of_snapshot_excludes_late_arrival() {
        let mut graph = EvidenceGraph::default();
        graph
            .append_node(node(
                "observation-late",
                NodeKind::SourceObservation,
                Some(100),
                None,
                300,
            ))
            .unwrap();

        let old_view = graph.snapshot_as_of(200);
        let current_view = graph.snapshot_as_of(400);
        assert!(!old_view.nodes.contains_key("observation-late"));
        assert!(current_view.nodes.contains_key("observation-late"));
        assert_eq!(
            current_view.nodes["observation-late"].valid_from,
            Some(100)
        );
    }

    #[test]
    fn later_revocation_is_not_backdated_into_prior_knowledge() {
        let mut graph = EvidenceGraph::default();
        graph
            .append_node(node(
                "authority-original",
                NodeKind::AuthorityAssertion,
                Some(100),
                Some(500),
                100,
            ))
            .unwrap();
        graph
            .append_node(node(
                "authority-revocation",
                NodeKind::AuthorityAssertion,
                Some(150),
                None,
                300,
            ))
            .unwrap();
        graph.append_edge(edge(
            "revokes-original",
            "authority-revocation",
            "authority-original",
            RelationKind::Revokes,
            Some(150),
            300,
        ));

        assert_eq!(graph.snapshot_as_of(200).edges.len(), 0);
        let later = graph.snapshot_as_of(400);
        assert_eq!(later.edges.len(), 1);
        assert_eq!(later.edges[0].valid_at, Some(150));
        assert_eq!(later.edges[0].recorded_at, 300);
    }

    #[test]
    fn exact_duplicate_node_is_idempotent_and_counted() {
        let mut graph = EvidenceGraph::default();
        let record = node(
            "observation-1",
            NodeKind::SourceObservation,
            Some(10),
            None,
            20,
        );
        assert!(graph.append_node(record.clone()).unwrap());
        assert!(!graph.append_node(record).unwrap());
        assert_eq!(graph.nodes.len(), 1);
        assert_eq!(graph.validate().exact_duplicate_nodes, 1);
    }

    #[test]
    fn conflicting_duplicate_identity_is_preserved_and_rejected() {
        let mut graph = EvidenceGraph::default();
        graph
            .append_node(node(
                "observation-1",
                NodeKind::SourceObservation,
                Some(10),
                None,
                20,
            ))
            .unwrap();
        let result = graph.append_node(node(
            "observation-1",
            NodeKind::SourceObservation,
            Some(11),
            None,
            21,
        ));
        assert_eq!(result.unwrap_err(), "conflicting-node-identity");
        assert_eq!(graph.node_conflicts.len(), 1);
        assert_eq!(graph.validate().conflicting_node_identities, 1);
    }

    #[test]
    fn dangling_and_wrongly_typed_edges_are_reported_separately() {
        let mut graph = EvidenceGraph::default();
        graph
            .append_node(node(
                "receipt-1",
                NodeKind::DeliveryReceipt,
                None,
                None,
                10,
            ))
            .unwrap();
        graph.append_edge(edge(
            "edge-dangling",
            "receipt-1",
            "missing-message",
            RelationKind::DeliveredAs,
            None,
            11,
        ));
        let dangling = graph.validate();
        assert_eq!(dangling.dangling_edges, 1);

        graph
            .append_node(node(
                "forecast-1",
                NodeKind::ForecastProduct,
                Some(1),
                None,
                10,
            ))
            .unwrap();
        graph.append_edge(edge(
            "edge-wrong-types",
            "receipt-1",
            "forecast-1",
            RelationKind::DeliveredAs,
            None,
            12,
        ));
        assert_eq!(graph.validate().invalid_relation_types, 1);
    }

    #[test]
    fn forward_reference_is_dangling_historically_and_resolves_later() {
        let mut graph = EvidenceGraph::default();
        graph.append_node(node("forecast-1", NodeKind::ForecastProduct, Some(10), None, 10)).unwrap();
        graph.append_edge(edge("derived-from-late-source", "forecast-1", "source-1", RelationKind::DerivedFrom, Some(10), 20));
        let historical = graph.snapshot_as_of(25);
        assert_eq!(historical.validate().dangling_edges, 1);
        graph.append_node(node("source-1", NodeKind::SourceObservation, Some(5), None, 30)).unwrap();
        let current = graph.validate();
        assert_eq!(current.dangling_edges, 0);
        assert_eq!(current.forward_reference_edges, 1);
        assert!(current.is_conformant());
    }

    #[test]
    fn forbidden_supersession_cycle_is_detected() {
        let mut graph = EvidenceGraph::default();
        for id in ["source-a", "source-b"] {
            graph
                .append_node(node(
                    id,
                    NodeKind::SourceObservation,
                    Some(1),
                    None,
                    2,
                ))
                .unwrap();
        }
        graph.append_edge(edge(
            "a-supersedes-b",
            "source-a",
            "source-b",
            RelationKind::Supersedes,
            Some(3),
            4,
        ));
        graph.append_edge(edge(
            "b-supersedes-a",
            "source-b",
            "source-a",
            RelationKind::Supersedes,
            Some(4),
            5,
        ));
        assert_eq!(graph.validate().forbidden_cycles, 1);
    }

    #[test]
    fn update_requires_an_exact_prior_reference() {
        let alert = cap_message(
            "alert-1",
            100,
            CapMessageType::Alert,
            vec![],
            Some(500),
            b"alert-v1",
        );
        let update = cap_message(
            "update-1",
            200,
            CapMessageType::Update,
            vec![alert.extended_identifier()],
            Some(600),
            b"alert-v2",
        );
        let audit = validate_cap_lifecycle(&[alert.clone(), update]);
        let report = audit.report;
        assert!(report.is_conformant());
        assert_eq!(report.updates, 1);
        assert_eq!(alert.wire_bytes, b"alert-v1");
    }

    #[test]
    fn update_without_reference_is_rejected() {
        let update = cap_message(
            "update-1",
            200,
            CapMessageType::Update,
            vec![],
            None,
            b"alert-v2",
        );
        let audit = validate_cap_lifecycle(&[update]);
        let report = audit.report;
        assert_eq!(report.missing_references, 1);
        assert!(!report.is_conformant());
    }

    #[test]
    fn unknown_non_prior_and_expired_references_are_distinguished() {
        let first = cap_message(
            "alert-1",
            100,
            CapMessageType::Alert,
            vec![],
            Some(150),
            b"alert-v1",
        );
        let expired_update = cap_message(
            "update-expired",
            200,
            CapMessageType::Update,
            vec![first.extended_identifier()],
            None,
            b"alert-v2",
        );
        let unknown_cancel = cap_message(
            "cancel-unknown",
            300,
            CapMessageType::Cancel,
            vec!["unknown,identifier,10".into()],
            None,
            b"cancel-v1",
        );
        let audit = validate_cap_lifecycle(&[first, expired_update, unknown_cancel]);
        let report = audit.report;
        assert_eq!(report.expired_references, 1);
        assert_eq!(report.unknown_references, 1);
        assert!(report.is_conformant());
        assert!(!report.is_conformant_under_active_reference_profile());
    }

    #[test]
    fn conflicting_cap_identifier_is_not_silently_deduplicated() {
        let original = cap_message(
            "alert-1",
            100,
            CapMessageType::Alert,
            vec![],
            Some(500),
            b"original",
        );
        let mut changed = original.clone();
        changed.wire_bytes = b"tampered".to_vec();
        let audit = validate_cap_lifecycle(&[original, changed]);
        let report = audit.report;
        assert_eq!(report.conflicting_identifiers, 1);
        assert_eq!(audit.conflicting_messages.len(), 1);
    }

    #[test]
    fn exact_cap_duplicate_is_idempotent_but_visible() {
        let original = cap_message(
            "alert-1",
            100,
            CapMessageType::Alert,
            vec![],
            Some(500),
            b"original",
        );
        let audit = validate_cap_lifecycle(&[original.clone(), original]);
        let report = audit.report;
        assert_eq!(report.exact_duplicates, 1);
        assert_eq!(report.conflicting_identifiers, 0);
    }

    #[test]
    fn receipt_must_bind_exact_message_digest() {
        let message = cap_message(
            "alert-1",
            100,
            CapMessageType::Alert,
            vec![],
            Some(500),
            b"message",
        );
        let exact = DeliveryReceipt {
            receipt_id: "receipt-1".into(),
            message_reference: message.extended_identifier(),
            message_digest: message.declared_digest.clone(),
            recorded_at: 110,
        };
        let mut wrong = exact.clone();
        wrong.receipt_id = "receipt-2".into();
        wrong.message_digest = "sha256:wrong".into();
        let report = validate_receipts(&[message], &[exact, wrong]);
        assert_eq!(report.exact_message_matches, 1);
        assert_eq!(report.digest_mismatches, 1);
        assert!(!report.is_conformant());
    }

    #[test]
    fn payload_digest_without_external_verification_is_not_a_pass() {
        let mut graph = EvidenceGraph::default();
        let mut record = node(
            "forecast-1",
            NodeKind::ForecastProduct,
            Some(1),
            None,
            2,
        );
        record.payload_digest = Some("sha256:fixture".into());
        record.digest_status = DigestStatus::Unchecked;
        graph.append_node(record).unwrap();
        assert_eq!(graph.validate().unverified_payload_digests, 1);
        assert!(!graph.validate().is_conformant());
    }

    #[test]
    fn receipt_cannot_select_one_of_conflicting_cap_payloads() {
        let original = cap_message(
            "alert-conflict",
            100,
            CapMessageType::Alert,
            vec![],
            Some(500),
            b"original",
        );
        let mut conflicting = original.clone();
        conflicting.wire_bytes = b"changed-payload".to_vec();
        conflicting.declared_digest = "sha256:other".into();
        let receipt = DeliveryReceipt {
            receipt_id: "receipt-ambiguous".into(),
            message_reference: original.extended_identifier(),
            message_digest: original.declared_digest.clone(),
            recorded_at: 110,
        };
        let report = validate_receipts(&[original, conflicting], &[receipt]);
        assert_eq!(report.conflicting_message_identities, 1);
        assert_eq!(report.receipts_for_ambiguous_messages, 1);
        assert_eq!(report.exact_message_matches, 0);
        assert!(!report.is_conformant());
    }

    #[test]
    fn invalid_validity_interval_fails() {
        let mut graph = EvidenceGraph::default();
        graph
            .append_node(node(
                "authority-1",
                NodeKind::AuthorityAssertion,
                Some(200),
                Some(100),
                10,
            ))
            .unwrap();
        assert_eq!(graph.validate().invalid_validity_intervals, 1);
    }
}
