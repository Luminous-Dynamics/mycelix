// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Persistence service for epistemic data
//!
//! JSON file-based storage for:
//! - Trust attestations
//! - Trust graph edges
//! - Contact trust history
//!
//! This avoids dependency conflicts with Holochain crates while
//! still providing persistent local storage.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter};
use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};
use thiserror::Error;

use crate::services::trust_graph::{RelationType, TrustAttestation, TrustEdge};

/// Database errors
#[derive(Error, Debug)]
pub enum PersistenceError {
    #[error("IO error: {0}")]
    IoError(String),

    #[error("Serialization error: {0}")]
    SerializationError(String),

    #[error("Not found: {0}")]
    NotFound(String),

    #[error("Lock error: {0}")]
    LockError(String),
}

impl From<std::io::Error> for PersistenceError {
    fn from(err: std::io::Error) -> Self {
        PersistenceError::IoError(err.to_string())
    }
}

impl From<serde_json::Error> for PersistenceError {
    fn from(err: serde_json::Error) -> Self {
        PersistenceError::SerializationError(err.to_string())
    }
}

pub type PersistenceResult<T> = Result<T, PersistenceError>;

/// Serializable trust attestation for storage
#[derive(Debug, Clone, Serialize, Deserialize)]
struct StoredAttestation {
    id: String,
    attestor_did: String,
    subject_did: String,
    message: String,
    relationship: String,
    trust_score: f64,
    created_at: String,
    expires_at: Option<String>,
    revoked: bool,
}

impl From<&TrustAttestation> for StoredAttestation {
    fn from(a: &TrustAttestation) -> Self {
        Self {
            id: a.id.clone(),
            attestor_did: a.attestor_did.clone(),
            subject_did: a.subject_did.clone(),
            message: a.message.clone(),
            relationship: relationship_to_string(&a.relationship).to_string(),
            trust_score: a.trust_score,
            created_at: a.created_at.to_rfc3339(),
            expires_at: a.expires_at.map(|dt| dt.to_rfc3339()),
            revoked: false,
        }
    }
}

impl StoredAttestation {
    fn to_attestation(&self) -> TrustAttestation {
        TrustAttestation {
            id: self.id.clone(),
            attestor_did: self.attestor_did.clone(),
            subject_did: self.subject_did.clone(),
            message: self.message.clone(),
            relationship: string_to_relationship(&self.relationship),
            trust_score: self.trust_score,
            created_at: parse_datetime(&self.created_at),
            expires_at: self.expires_at.as_ref().map(|s| parse_datetime(s)),
        }
    }
}

/// Serializable trust edge for storage
#[derive(Debug, Clone, Serialize, Deserialize)]
struct StoredEdge {
    from_did: String,
    to_did: String,
    relationship: String,
    trust_score: f64,
    reason: Option<String>,
    established_at: String,
    decays_at: Option<String>,
}

impl StoredEdge {
    fn to_edge(&self) -> TrustEdge {
        TrustEdge {
            from_did: self.from_did.clone(),
            to_did: self.to_did.clone(),
            relationship: string_to_relationship(&self.relationship),
            trust_score: self.trust_score,
            reason: self.reason.clone(),
            established_at: parse_datetime(&self.established_at),
            decays_at: self.decays_at.as_ref().map(|s| parse_datetime(s)),
        }
    }
}

/// Trust history entry
#[derive(Debug, Clone, Serialize, Deserialize)]
struct TrustHistoryEntry {
    trust_score: f64,
    recorded_at: String,
}

/// The full data store
#[derive(Debug, Default, Serialize, Deserialize)]
struct DataStore {
    attestations: HashMap<String, StoredAttestation>,
    edges: HashMap<String, StoredEdge>, // key: "from_did|to_did|relationship"
    trust_history: HashMap<String, Vec<TrustHistoryEntry>>, // key: "user_did|contact_did"
}

/// Persistence service for epistemic data
#[derive(Clone)]
pub struct PersistenceService {
    data_path: Option<PathBuf>,
    store: Arc<RwLock<DataStore>>,
}

impl PersistenceService {
    /// Create a new persistence service with file backing
    pub fn new(data_path: &Path) -> PersistenceResult<Self> {
        let store = if data_path.exists() {
            let file = File::open(data_path)?;
            let reader = BufReader::new(file);
            serde_json::from_reader(reader).unwrap_or_default()
        } else {
            DataStore::default()
        };

        Ok(Self {
            data_path: Some(data_path.to_path_buf()),
            store: Arc::new(RwLock::new(store)),
        })
    }

    /// Create an in-memory store (for testing)
    pub fn in_memory() -> PersistenceResult<Self> {
        Ok(Self {
            data_path: None,
            store: Arc::new(RwLock::new(DataStore::default())),
        })
    }

    /// Save the store to disk
    fn save(&self) -> PersistenceResult<()> {
        if let Some(path) = &self.data_path {
            let store = self.store.read()
                .map_err(|e| PersistenceError::LockError(e.to_string()))?;

            // Ensure parent directory exists
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent)?;
            }

            let file = File::create(path)?;
            let writer = BufWriter::new(file);
            serde_json::to_writer_pretty(writer, &*store)?;
        }
        Ok(())
    }

    // ============================================
    // Trust Attestation Operations
    // ============================================

    /// Store a trust attestation
    pub fn store_attestation(&self, attestation: &TrustAttestation) -> PersistenceResult<()> {
        {
            let mut store = self.store.write()
                .map_err(|e| PersistenceError::LockError(e.to_string()))?;

            // Store attestation
            let stored = StoredAttestation::from(attestation);
            store.attestations.insert(attestation.id.clone(), stored);

            // Update edge
            let edge_key = format!(
                "{}|{}|{}",
                attestation.attestor_did,
                attestation.subject_did,
                relationship_to_string(&attestation.relationship)
            );
            store.edges.insert(edge_key, StoredEdge {
                from_did: attestation.attestor_did.clone(),
                to_did: attestation.subject_did.clone(),
                relationship: relationship_to_string(&attestation.relationship).to_string(),
                trust_score: attestation.trust_score,
                reason: Some(attestation.message.clone()),
                established_at: attestation.created_at.to_rfc3339(),
                decays_at: attestation.expires_at.map(|dt| dt.to_rfc3339()),
            });
        }

        self.save()
    }

    /// Get an attestation by ID
    pub fn get_attestation(&self, id: &str) -> PersistenceResult<TrustAttestation> {
        let store = self.store.read()
            .map_err(|e| PersistenceError::LockError(e.to_string()))?;

        store.attestations.get(id)
            .filter(|a| !a.revoked)
            .map(|a| a.to_attestation())
            .ok_or_else(|| PersistenceError::NotFound(format!("Attestation {} not found", id)))
    }

    /// Get attestations by attestor
    pub fn get_attestations_by_attestor(&self, attestor_did: &str) -> PersistenceResult<Vec<TrustAttestation>> {
        let store = self.store.read()
            .map_err(|e| PersistenceError::LockError(e.to_string()))?;

        Ok(store.attestations.values()
            .filter(|a| a.attestor_did == attestor_did && !a.revoked)
            .map(|a| a.to_attestation())
            .collect())
    }

    /// Get attestations for subject
    pub fn get_attestations_for_subject(&self, subject_did: &str) -> PersistenceResult<Vec<TrustAttestation>> {
        let store = self.store.read()
            .map_err(|e| PersistenceError::LockError(e.to_string()))?;

        Ok(store.attestations.values()
            .filter(|a| a.subject_did == subject_did && !a.revoked)
            .map(|a| a.to_attestation())
            .collect())
    }

    /// Revoke an attestation
    pub fn revoke_attestation(&self, id: &str) -> PersistenceResult<()> {
        {
            let mut store = self.store.write()
                .map_err(|e| PersistenceError::LockError(e.to_string()))?;

            let attestation = store.attestations.get_mut(id)
                .ok_or_else(|| PersistenceError::NotFound(format!("Attestation {} not found", id)))?;

            attestation.revoked = true;
        }

        self.save()
    }

    // ============================================
    // Trust Edge Operations
    // ============================================

    /// Get outgoing edges from a DID
    pub fn get_outgoing_edges(&self, from_did: &str) -> PersistenceResult<Vec<TrustEdge>> {
        let store = self.store.read()
            .map_err(|e| PersistenceError::LockError(e.to_string()))?;

        Ok(store.edges.values()
            .filter(|e| e.from_did == from_did)
            .map(|e| e.to_edge())
            .collect())
    }

    /// Get incoming edges to a DID
    pub fn get_incoming_edges(&self, to_did: &str) -> PersistenceResult<Vec<TrustEdge>> {
        let store = self.store.read()
            .map_err(|e| PersistenceError::LockError(e.to_string()))?;

        Ok(store.edges.values()
            .filter(|e| e.to_did == to_did)
            .map(|e| e.to_edge())
            .collect())
    }

    // ============================================
    // Contact Trust History
    // ============================================

    /// Record trust score for a contact
    pub fn record_trust_history(
        &self,
        user_did: &str,
        contact_did: &str,
        trust_score: f64,
    ) -> PersistenceResult<()> {
        {
            let mut store = self.store.write()
                .map_err(|e| PersistenceError::LockError(e.to_string()))?;

            let key = format!("{}|{}", user_did, contact_did);
            let history = store.trust_history.entry(key).or_default();

            history.push(TrustHistoryEntry {
                trust_score,
                recorded_at: chrono::Utc::now().to_rfc3339(),
            });

            // Keep only last 100 entries per contact
            if history.len() > 100 {
                history.remove(0);
            }
        }

        self.save()
    }

    /// Get trust history for a contact
    pub fn get_trust_history(
        &self,
        user_did: &str,
        contact_did: &str,
        limit: usize,
    ) -> PersistenceResult<Vec<(f64, chrono::DateTime<chrono::Utc>)>> {
        let store = self.store.read()
            .map_err(|e| PersistenceError::LockError(e.to_string()))?;

        let key = format!("{}|{}", user_did, contact_did);
        Ok(store.trust_history.get(&key)
            .map(|history| {
                history.iter()
                    .rev()
                    .take(limit)
                    .map(|e| (e.trust_score, parse_datetime(&e.recorded_at)))
                    .collect()
            })
            .unwrap_or_default())
    }

    /// Get all contacts with trust history for a user
    pub fn get_contacts_with_history(&self, user_did: &str) -> PersistenceResult<Vec<String>> {
        let store = self.store.read()
            .map_err(|e| PersistenceError::LockError(e.to_string()))?;

        let prefix = format!("{}|", user_did);
        Ok(store.trust_history.keys()
            .filter(|k| k.starts_with(&prefix))
            .map(|k| k.strip_prefix(&prefix).unwrap_or("").to_string())
            .collect())
    }
}

// Helper functions
fn relationship_to_string(rel: &RelationType) -> &'static str {
    match rel {
        RelationType::DirectTrust => "direct_trust",
        RelationType::Introduction => "introduction",
        RelationType::OrganizationMember => "organization_member",
        RelationType::CredentialIssuer => "credential_issuer",
        RelationType::TransitiveTrust => "transitive_trust",
        RelationType::Vouch => "vouch",
    }
}

fn string_to_relationship(s: &str) -> RelationType {
    match s {
        "direct_trust" => RelationType::DirectTrust,
        "introduction" => RelationType::Introduction,
        "organization_member" => RelationType::OrganizationMember,
        "credential_issuer" => RelationType::CredentialIssuer,
        "transitive_trust" => RelationType::TransitiveTrust,
        "vouch" => RelationType::Vouch,
        _ => RelationType::TransitiveTrust,
    }
}

fn parse_datetime(s: &str) -> chrono::DateTime<chrono::Utc> {
    chrono::DateTime::parse_from_rfc3339(s)
        .map(|dt| dt.with_timezone(&chrono::Utc))
        .unwrap_or_else(|_| chrono::Utc::now())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_attestation_crud() {
        let service = PersistenceService::in_memory().unwrap();

        let attestation = TrustAttestation {
            id: "test-1".to_string(),
            attestor_did: "did:mycelix:alice".to_string(),
            subject_did: "did:mycelix:bob".to_string(),
            message: "I trust Bob".to_string(),
            relationship: RelationType::DirectTrust,
            trust_score: 0.8,
            created_at: chrono::Utc::now(),
            expires_at: None,
        };

        service.store_attestation(&attestation).unwrap();

        let retrieved = service.get_attestation("test-1").unwrap();
        assert_eq!(retrieved.attestor_did, "did:mycelix:alice");
        assert_eq!(retrieved.trust_score, 0.8);

        service.revoke_attestation("test-1").unwrap();
        assert!(service.get_attestation("test-1").is_err());
    }

    #[test]
    fn test_trust_history() {
        let service = PersistenceService::in_memory().unwrap();

        service.record_trust_history("did:mycelix:alice", "did:mycelix:bob", 0.5).unwrap();
        service.record_trust_history("did:mycelix:alice", "did:mycelix:bob", 0.7).unwrap();

        let history = service.get_trust_history("did:mycelix:alice", "did:mycelix:bob", 10).unwrap();
        assert_eq!(history.len(), 2);
        assert_eq!(history[0].0, 0.7); // Most recent first
    }

    #[test]
    fn test_edges() {
        let service = PersistenceService::in_memory().unwrap();

        let attestation = TrustAttestation {
            id: "test-edge".to_string(),
            attestor_did: "did:mycelix:alice".to_string(),
            subject_did: "did:mycelix:bob".to_string(),
            message: "Trust".to_string(),
            relationship: RelationType::DirectTrust,
            trust_score: 0.9,
            created_at: chrono::Utc::now(),
            expires_at: None,
        };

        service.store_attestation(&attestation).unwrap();

        let outgoing = service.get_outgoing_edges("did:mycelix:alice").unwrap();
        assert_eq!(outgoing.len(), 1);
        assert_eq!(outgoing[0].to_did, "did:mycelix:bob");

        let incoming = service.get_incoming_edges("did:mycelix:bob").unwrap();
        assert_eq!(incoming.len(), 1);
        assert_eq!(incoming[0].from_did, "did:mycelix:alice");
    }
}
