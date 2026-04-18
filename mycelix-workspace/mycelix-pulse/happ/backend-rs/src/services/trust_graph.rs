// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Trust Graph Service
//!
//! Manages the social trust fabric - showing not just trust scores,
//! but the paths and reasons behind trust relationships.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;
use thiserror::Error;
use tokio::sync::RwLock;
use utoipa::ToSchema;

/// Errors from trust graph operations
#[derive(Debug, Error)]
pub enum TrustGraphError {
    #[error("DID not found in graph: {0}")]
    DidNotFound(String),

    #[error("No path exists between DIDs")]
    NoPathExists,

    #[error("Graph operation failed: {0}")]
    OperationFailed(String),
}

/// Type of relationship between two parties
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash, ToSchema)]
#[serde(rename_all = "snake_case")]
pub enum RelationType {
    /// Direct personal trust
    DirectTrust,
    /// Introduction from a trusted party
    Introduction,
    /// Same organization membership
    OrganizationMember,
    /// Credential issuer relationship
    CredentialIssuer,
    /// Transitive trust through path
    TransitiveTrust,
    /// Vouching relationship
    Vouch,
}

impl RelationType {
    /// Trust decay factor for this relationship type
    pub fn decay_factor(&self) -> f64 {
        match self {
            Self::DirectTrust => 0.95,       // Minimal decay
            Self::CredentialIssuer => 0.90,  // Slightly more
            Self::OrganizationMember => 0.85,
            Self::Vouch => 0.80,
            Self::Introduction => 0.75,
            Self::TransitiveTrust => 0.70,   // Most decay
        }
    }
}

/// A node in the trust graph (a person/entity)
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct TrustNode {
    /// The DID of this entity
    pub did: String,

    /// Human-readable name (if known)
    pub name: Option<String>,

    /// Organization affiliation (if known)
    pub organization: Option<String>,

    /// When this node was added to the graph
    pub added_at: DateTime<Utc>,

    /// Number of outgoing trust edges
    pub trusts_count: usize,

    /// Number of incoming trust edges
    pub trusted_by_count: usize,
}

/// An edge in the trust graph (a trust relationship)
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct TrustEdge {
    /// The trusting party's DID
    pub from_did: String,

    /// The trusted party's DID
    pub to_did: String,

    /// Type of relationship
    pub relationship: RelationType,

    /// Trust score (0.0 - 1.0)
    pub trust_score: f64,

    /// Reason for this trust (optional)
    pub reason: Option<String>,

    /// When this trust was established
    pub established_at: DateTime<Utc>,

    /// When this trust expires/decays significantly
    pub decays_at: Option<DateTime<Utc>>,
}

/// A single hop in a trust path
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct TrustHop {
    /// From DID
    pub from: String,

    /// To DID
    pub to: String,

    /// Name of the 'from' party
    pub from_name: Option<String>,

    /// Name of the 'to' party
    pub to_name: Option<String>,

    /// Relationship type
    pub relationship: RelationType,

    /// Trust score for this hop
    pub trust_score: f64,

    /// Reason for this hop's trust
    pub reason: Option<String>,
}

/// A complete path through the trust graph
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct TrustPath {
    /// The hops in the path
    pub hops: Vec<TrustHop>,

    /// Cumulative trust after decay
    pub final_trust: f64,

    /// Number of hops
    pub path_length: usize,

    /// The strongest link in the path
    pub strongest_link: Option<TrustHop>,

    /// The weakest link in the path
    pub weakest_link: Option<TrustHop>,
}

impl TrustPath {
    /// Calculate cumulative trust with decay
    pub fn calculate_trust(&self) -> f64 {
        if self.hops.is_empty() {
            return 0.0;
        }

        let mut trust = 1.0;
        for hop in &self.hops {
            trust *= hop.trust_score * hop.relationship.decay_factor();
        }

        trust
    }

    /// Find strongest and weakest links
    pub fn analyze_links(&mut self) {
        if let Some(strongest) = self.hops.iter().max_by(|a, b| {
            a.trust_score
                .partial_cmp(&b.trust_score)
                .unwrap_or(core::cmp::Ordering::Equal)
        }) {
            self.strongest_link = Some(strongest.clone());
        }

        if let Some(weakest) = self.hops.iter().min_by(|a, b| {
            a.trust_score
                .partial_cmp(&b.trust_score)
                .unwrap_or(core::cmp::Ordering::Equal)
        }) {
            self.weakest_link = Some(weakest.clone());
        }
    }
}

/// The full trust graph
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct TrustGraph {
    /// All nodes in the graph
    pub nodes: Vec<TrustNode>,

    /// All edges in the graph
    pub edges: Vec<TrustEdge>,

    /// User's DID (center of the graph)
    pub center_did: String,

    /// Maximum path length included
    pub max_depth: usize,
}

/// An attestation of trust
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct TrustAttestation {
    /// Unique ID
    pub id: String,

    /// Who is attesting
    pub attestor_did: String,

    /// Who is being attested
    pub subject_did: String,

    /// The attestation message
    pub message: String,

    /// Type of relationship established
    pub relationship: RelationType,

    /// Trust score assigned
    pub trust_score: f64,

    /// When created
    pub created_at: DateTime<Utc>,

    /// When this expires
    pub expires_at: Option<DateTime<Utc>>,
}

/// Service for managing the trust graph
pub struct TrustGraphService {
    /// Edges indexed by from_did
    edges_from: Arc<RwLock<HashMap<String, Vec<TrustEdge>>>>,

    /// Edges indexed by to_did
    edges_to: Arc<RwLock<HashMap<String, Vec<TrustEdge>>>>,

    /// Nodes by DID
    nodes: Arc<RwLock<HashMap<String, TrustNode>>>,

    /// Maximum path length to search
    max_path_length: usize,
}

impl TrustGraphService {
    pub fn new() -> Self {
        Self {
            edges_from: Arc::new(RwLock::new(HashMap::new())),
            edges_to: Arc::new(RwLock::new(HashMap::new())),
            nodes: Arc::new(RwLock::new(HashMap::new())),
            max_path_length: 5,
        }
    }

    /// Add or update a node
    pub async fn upsert_node(&self, node: TrustNode) {
        let mut nodes = self.nodes.write().await;
        nodes.insert(node.did.clone(), node);
    }

    /// Add a trust edge
    pub async fn add_edge(&self, edge: TrustEdge) {
        let from_did = edge.from_did.clone();
        let to_did = edge.to_did.clone();

        // Add to from index
        {
            let mut edges_from = self.edges_from.write().await;
            edges_from.entry(from_did.clone())
                .or_insert_with(Vec::new)
                .push(edge.clone());
        }

        // Add to to index
        {
            let mut edges_to = self.edges_to.write().await;
            edges_to.entry(to_did.clone())
                .or_insert_with(Vec::new)
                .push(edge);
        }

        // Update node counts
        {
            let mut nodes = self.nodes.write().await;
            if let Some(from_node) = nodes.get_mut(&from_did) {
                from_node.trusts_count += 1;
            }
            if let Some(to_node) = nodes.get_mut(&to_did) {
                to_node.trusted_by_count += 1;
            }
        }
    }

    /// Find the shortest trust path between two DIDs
    pub async fn find_path(&self, from_did: &str, to_did: &str) -> Result<TrustPath, TrustGraphError> {
        let edges_from = self.edges_from.read().await;
        let nodes = self.nodes.read().await;

        // BFS to find shortest path
        let mut queue: VecDeque<(String, Vec<TrustHop>)> = VecDeque::new();
        let mut visited: HashSet<String> = HashSet::new();

        queue.push_back((from_did.to_string(), Vec::new()));
        visited.insert(from_did.to_string());

        while let Some((current, path)) = queue.pop_front() {
            if current == to_did {
                let mut trust_path = TrustPath {
                    hops: path.clone(),
                    final_trust: 0.0,
                    path_length: path.len(),
                    strongest_link: None,
                    weakest_link: None,
                };
                trust_path.final_trust = trust_path.calculate_trust();
                trust_path.analyze_links();
                return Ok(trust_path);
            }

            if path.len() >= self.max_path_length {
                continue;
            }

            if let Some(edges) = edges_from.get(&current) {
                for edge in edges {
                    if !visited.contains(&edge.to_did) {
                        visited.insert(edge.to_did.clone());

                        let from_node = nodes.get(&edge.from_did);
                        let to_node = nodes.get(&edge.to_did);

                        let hop = TrustHop {
                            from: edge.from_did.clone(),
                            to: edge.to_did.clone(),
                            from_name: from_node.and_then(|n| n.name.clone()),
                            to_name: to_node.and_then(|n| n.name.clone()),
                            relationship: edge.relationship.clone(),
                            trust_score: edge.trust_score,
                            reason: edge.reason.clone(),
                        };

                        let mut new_path = path.clone();
                        new_path.push(hop);
                        queue.push_back((edge.to_did.clone(), new_path));
                    }
                }
            }
        }

        Err(TrustGraphError::NoPathExists)
    }

    /// Get the trust subgraph centered on a DID
    pub async fn get_subgraph(&self, center_did: &str, depth: usize) -> TrustGraph {
        let edges_from = self.edges_from.read().await;
        let nodes = self.nodes.read().await;

        let mut visited: HashSet<String> = HashSet::new();
        let mut queue: VecDeque<(String, usize)> = VecDeque::new();
        let mut result_nodes = Vec::new();
        let mut result_edges = Vec::new();

        queue.push_back((center_did.to_string(), 0));
        visited.insert(center_did.to_string());

        while let Some((current, current_depth)) = queue.pop_front() {
            if let Some(node) = nodes.get(&current) {
                result_nodes.push(node.clone());
            }

            if current_depth >= depth {
                continue;
            }

            if let Some(edges) = edges_from.get(&current) {
                for edge in edges {
                    result_edges.push(edge.clone());

                    if !visited.contains(&edge.to_did) {
                        visited.insert(edge.to_did.clone());
                        queue.push_back((edge.to_did.clone(), current_depth + 1));
                    }
                }
            }
        }

        TrustGraph {
            nodes: result_nodes,
            edges: result_edges,
            center_did: center_did.to_string(),
            max_depth: depth,
        }
    }

    /// Create an attestation of trust
    pub async fn create_attestation(&self, attestation: TrustAttestation) -> Result<(), TrustGraphError> {
        // Create the edge from the attestation
        let edge = TrustEdge {
            from_did: attestation.attestor_did.clone(),
            to_did: attestation.subject_did.clone(),
            relationship: attestation.relationship.clone(),
            trust_score: attestation.trust_score,
            reason: Some(attestation.message.clone()),
            established_at: attestation.created_at,
            decays_at: attestation.expires_at,
        };

        self.add_edge(edge).await;

        // Ensure both nodes exist
        let now = Utc::now();
        self.upsert_node(TrustNode {
            did: attestation.attestor_did.clone(),
            name: None,
            organization: None,
            added_at: now,
            trusts_count: 0,
            trusted_by_count: 0,
        }).await;

        self.upsert_node(TrustNode {
            did: attestation.subject_did.clone(),
            name: None,
            organization: None,
            added_at: now,
            trusts_count: 0,
            trusted_by_count: 0,
        }).await;

        Ok(())
    }

    /// Get who a DID trusts
    pub async fn get_trusted_by(&self, did: &str) -> Vec<TrustEdge> {
        let edges_from = self.edges_from.read().await;
        edges_from.get(did).cloned().unwrap_or_default()
    }

    /// Get who trusts a DID
    pub async fn get_trusters(&self, did: &str) -> Vec<TrustEdge> {
        let edges_to = self.edges_to.read().await;
        edges_to.get(did).cloned().unwrap_or_default()
    }

    /// Calculate aggregate trust from all paths
    pub async fn calculate_aggregate_trust(&self, from_did: &str, to_did: &str) -> f64 {
        // For now, just use the shortest path
        // In production: combine multiple paths with diminishing returns
        match self.find_path(from_did, to_did).await {
            Ok(path) => path.final_trust,
            Err(_) => 0.0,
        }
    }
}

impl Default for TrustGraphService {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_direct_trust_path() {
        let service = TrustGraphService::new();

        // Alice trusts Bob directly
        service.add_edge(TrustEdge {
            from_did: "did:mycelix:alice".into(),
            to_did: "did:mycelix:bob".into(),
            relationship: RelationType::DirectTrust,
            trust_score: 0.9,
            reason: Some("Worked together".into()),
            established_at: Utc::now(),
            decays_at: None,
        }).await;

        let path = service.find_path("did:mycelix:alice", "did:mycelix:bob").await.unwrap();
        assert_eq!(path.path_length, 1);
        assert!(path.final_trust > 0.8);
    }

    #[tokio::test]
    async fn test_transitive_trust_path() {
        let service = TrustGraphService::new();

        // Alice trusts Bob
        service.add_edge(TrustEdge {
            from_did: "did:mycelix:alice".into(),
            to_did: "did:mycelix:bob".into(),
            relationship: RelationType::DirectTrust,
            trust_score: 0.9,
            reason: None,
            established_at: Utc::now(),
            decays_at: None,
        }).await;

        // Bob trusts Charlie
        service.add_edge(TrustEdge {
            from_did: "did:mycelix:bob".into(),
            to_did: "did:mycelix:charlie".into(),
            relationship: RelationType::Vouch,
            trust_score: 0.8,
            reason: None,
            established_at: Utc::now(),
            decays_at: None,
        }).await;

        let path = service.find_path("did:mycelix:alice", "did:mycelix:charlie").await.unwrap();
        assert_eq!(path.path_length, 2);
        // Trust decays over hops
        assert!(path.final_trust < 0.9 * 0.8);
    }

    #[tokio::test]
    async fn test_no_path() {
        let service = TrustGraphService::new();

        let result = service.find_path("did:mycelix:alice", "did:mycelix:stranger").await;
        assert!(matches!(result, Err(TrustGraphError::NoPathExists)));
    }

    #[tokio::test]
    async fn test_subgraph() {
        let service = TrustGraphService::new();

        // Create a small network
        service.add_edge(TrustEdge {
            from_did: "did:mycelix:alice".into(),
            to_did: "did:mycelix:bob".into(),
            relationship: RelationType::DirectTrust,
            trust_score: 0.9,
            reason: None,
            established_at: Utc::now(),
            decays_at: None,
        }).await;

        service.add_edge(TrustEdge {
            from_did: "did:mycelix:alice".into(),
            to_did: "did:mycelix:charlie".into(),
            relationship: RelationType::DirectTrust,
            trust_score: 0.85,
            reason: None,
            established_at: Utc::now(),
            decays_at: None,
        }).await;

        let graph = service.get_subgraph("did:mycelix:alice", 1).await;
        assert_eq!(graph.edges.len(), 2);
    }

    #[test]
    fn test_relationship_decay() {
        assert!(RelationType::DirectTrust.decay_factor() > RelationType::TransitiveTrust.decay_factor());
    }
}
