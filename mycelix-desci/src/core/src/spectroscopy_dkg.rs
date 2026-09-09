// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Symthaea spectroscopy ↔ Mycelix epistemic-DKG bridge.
//!
//! The transport schema is deliberately dependency-light: Mycelix does not
//! depend on the Symthaea crate. Symthaea exports a canonical bundle of claims,
//! evidence, provenance and relations; Mycelix validates and projects those
//! records into its epistemic graph semantics.
//!
//! Invariants:
//! - observations/evidence are append-only records;
//! - claims are separate from evidence artifacts;
//! - support/challenge/supersession is represented as edges;
//! - consensus is derived downstream, never written into historical evidence;
//! - source provenance remains attached to every imported evidence record.

use crate::claims::ClaimRelationType;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

pub const SPECTROSCOPY_DKG_PROTOCOL: &str = "mycelix-symthaea-spectroscopy";
pub const SPECTROSCOPY_DKG_SCHEMA_VERSION: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpectroscopyNodeKind {
    Claim,
    Observation,
    UpperLimit,
    LatticePrediction,
    PhenomenologicalModel,
    Replication,
    Contradiction,
    SupersedingEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpectroscopyRelation {
    Supports,
    Challenges,
    Constrains,
    Replicates,
    Supersedes,
    ContextFor,
}

impl SpectroscopyRelation {
    /// Loss-minimizing projection into the existing Mycelix claim graph.
    /// The original relation is retained in the imported bundle, so this
    /// projection never destroys the richer spectroscopy semantics.
    pub fn claim_relation_type(self) -> ClaimRelationType {
        match self {
            Self::Supports | Self::Replicates => ClaimRelationType::Supports,
            Self::Challenges => ClaimRelationType::Refutes,
            Self::Supersedes => ClaimRelationType::Supercedes,
            Self::Constrains => ClaimRelationType::Restricts,
            Self::ContextFor => ClaimRelationType::Cites,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SpectroscopyProvenance {
    pub source: String,
    /// DOI, arXiv identifier, collaboration release, dataset PID, etc.
    pub persistent_id: String,
    /// Optional content hash supplied by the producer for byte-level integrity.
    pub content_hash: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SpectroscopyDkgNode {
    pub id: String,
    pub kind: SpectroscopyNodeKind,
    pub subject: String,
    /// Claim statement, observation summary, prediction summary, etc.
    pub statement: String,
    /// Stable observable key when the node concerns an observable.
    pub observable: Option<String>,
    pub provenance: Option<SpectroscopyProvenance>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SpectroscopyDkgEdge {
    pub from: String,
    pub relation: SpectroscopyRelation,
    pub to: String,
    pub rationale: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SpectroscopyDkgBundle {
    pub protocol: String,
    pub schema_version: u16,
    /// Producer identifier, e.g. `symthaea` plus a release/evidence lineage.
    pub producer: String,
    pub nodes: Vec<SpectroscopyDkgNode>,
    pub edges: Vec<SpectroscopyDkgEdge>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SpectroscopyDkgError {
    ProtocolMismatch,
    UnsupportedSchema(u16),
    EmptyProducer,
    EmptyId,
    DuplicateId(String),
    MissingNode(String),
    SelfEdge(String),
    EvidenceMissingProvenance(String),
}

impl SpectroscopyDkgBundle {
    pub fn validate(&self) -> Result<(), SpectroscopyDkgError> {
        if self.protocol != SPECTROSCOPY_DKG_PROTOCOL {
            return Err(SpectroscopyDkgError::ProtocolMismatch);
        }
        if self.schema_version != SPECTROSCOPY_DKG_SCHEMA_VERSION {
            return Err(SpectroscopyDkgError::UnsupportedSchema(self.schema_version));
        }
        if self.producer.trim().is_empty() {
            return Err(SpectroscopyDkgError::EmptyProducer);
        }

        let mut ids = HashSet::new();
        for node in &self.nodes {
            if node.id.trim().is_empty() {
                return Err(SpectroscopyDkgError::EmptyId);
            }
            if !ids.insert(node.id.as_str()) {
                return Err(SpectroscopyDkgError::DuplicateId(node.id.clone()));
            }
            if node.kind != SpectroscopyNodeKind::Claim && node.provenance.is_none() {
                return Err(SpectroscopyDkgError::EvidenceMissingProvenance(
                    node.id.clone(),
                ));
            }
        }

        for edge in &self.edges {
            if edge.from == edge.to {
                return Err(SpectroscopyDkgError::SelfEdge(edge.from.clone()));
            }
            if !ids.contains(edge.from.as_str()) {
                return Err(SpectroscopyDkgError::MissingNode(edge.from.clone()));
            }
            if !ids.contains(edge.to.as_str()) {
                return Err(SpectroscopyDkgError::MissingNode(edge.to.clone()));
            }
        }
        Ok(())
    }

    pub fn claims(&self) -> impl Iterator<Item = &SpectroscopyDkgNode> {
        self.nodes
            .iter()
            .filter(|node| node.kind == SpectroscopyNodeKind::Claim)
    }

    pub fn evidence(&self) -> impl Iterator<Item = &SpectroscopyDkgNode> {
        self.nodes
            .iter()
            .filter(|node| node.kind != SpectroscopyNodeKind::Claim)
    }

    /// Return all evidence/support/challenge context directly incident on a
    /// claim without collapsing the edge relation into a scalar confidence.
    pub fn context_for_claim(
        &self,
        claim_id: &str,
    ) -> Vec<(&SpectroscopyDkgNode, SpectroscopyRelation)> {
        self.edges
            .iter()
            .filter_map(|edge| {
                if edge.to != claim_id {
                    return None;
                }
                self.nodes
                    .iter()
                    .find(|node| node.id == edge.from)
                    .map(|node| (node, edge.relation))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bundle() -> SpectroscopyDkgBundle {
        SpectroscopyDkgBundle {
            protocol: SPECTROSCOPY_DKG_PROTOCOL.into(),
            schema_version: SPECTROSCOPY_DKG_SCHEMA_VERSION,
            producer: "symthaea:test-lineage".into(),
            nodes: vec![
                SpectroscopyDkgNode {
                    id: "claim::x2370::glueball".into(),
                    kind: SpectroscopyNodeKind::Claim,
                    subject: "X(2370)".into(),
                    statement: "X(2370) has a dominant pseudoscalar-glueball component".into(),
                    observable: None,
                    provenance: None,
                },
                SpectroscopyDkgNode {
                    id: "evidence::x2370::jpc".into(),
                    kind: SpectroscopyNodeKind::Observation,
                    subject: "X(2370)".into(),
                    statement: "J^PC = 0-+".into(),
                    observable: Some("jpc".into()),
                    provenance: Some(SpectroscopyProvenance {
                        source: "BESIII".into(),
                        persistent_id: "arXiv:2312.05324".into(),
                        content_hash: None,
                    }),
                },
                SpectroscopyDkgNode {
                    id: "model::x2370::molecule".into(),
                    kind: SpectroscopyNodeKind::PhenomenologicalModel,
                    subject: "X(2370)".into(),
                    statement: "Hadronic-molecule-family alternative".into(),
                    observable: None,
                    provenance: Some(SpectroscopyProvenance {
                        source: "theory".into(),
                        persistent_id: "model-family".into(),
                        content_hash: None,
                    }),
                },
            ],
            edges: vec![
                SpectroscopyDkgEdge {
                    from: "evidence::x2370::jpc".into(),
                    relation: SpectroscopyRelation::Supports,
                    to: "claim::x2370::glueball".into(),
                    rationale: None,
                },
                SpectroscopyDkgEdge {
                    from: "model::x2370::molecule".into(),
                    relation: SpectroscopyRelation::Challenges,
                    to: "claim::x2370::glueball".into(),
                    rationale: Some("competing internal-composition model".into()),
                },
            ],
        }
    }

    #[test]
    fn validates_and_preserves_competing_evidence() {
        let bundle = bundle();
        assert!(bundle.validate().is_ok());
        let context = bundle.context_for_claim("claim::x2370::glueball");
        assert_eq!(context.len(), 2);
        assert!(context.iter().any(|(_, r)| *r == SpectroscopyRelation::Supports));
        assert!(context.iter().any(|(_, r)| *r == SpectroscopyRelation::Challenges));
    }

    #[test]
    fn evidence_requires_provenance() {
        let mut bundle = bundle();
        bundle.nodes[1].provenance = None;
        assert!(matches!(
            bundle.validate(),
            Err(SpectroscopyDkgError::EvidenceMissingProvenance(_))
        ));
    }

    #[test]
    fn rich_relations_project_into_existing_claim_graph_semantics() {
        assert_eq!(
            SpectroscopyRelation::Supersedes.claim_relation_type(),
            ClaimRelationType::Supercedes
        );
        assert_eq!(
            SpectroscopyRelation::Challenges.claim_relation_type(),
            ClaimRelationType::Refutes
        );
        assert_eq!(
            SpectroscopyRelation::Constrains.claim_relation_type(),
            ClaimRelationType::Restricts
        );
    }
}
