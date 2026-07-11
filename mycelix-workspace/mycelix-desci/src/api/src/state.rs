// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Application state shared across handlers

#[cfg(feature = "holochain")]
use mycelix_desci_core::knowledge_storage::KnowledgeDhtStorage;
use mycelix_desci_core::{
    query::QueryEngine,
    storage::{MemoryStorage, StorageBackend},
    trust::TrustManager,
};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::{Duration, Instant};
use tokio::sync::RwLock;

/// Application metrics
#[derive(Clone)]
pub struct Metrics {
    pub queries_executed: Arc<AtomicU64>,
    pub claims_created: Arc<AtomicU64>,
    pub verifications_added: Arc<AtomicU64>,
    response_times: Arc<RwLock<Vec<u64>>>,
}

impl Metrics {
    fn new() -> Self {
        Self {
            queries_executed: Arc::new(AtomicU64::new(0)),
            claims_created: Arc::new(AtomicU64::new(0)),
            verifications_added: Arc::new(AtomicU64::new(0)),
            response_times: Arc::new(RwLock::new(Vec::new())),
        }
    }

    pub fn average_response_time_ms(&self) -> f64 {
        // This would be more sophisticated in production
        0.0
    }
}

/// Shared application state
#[derive(Clone)]
pub struct AppState {
    /// Storage backend. Selected at startup via `DESCI_STORAGE_BACKEND`
    /// (`memory` [default] | `knowledge_dht`) -- see `AppState::new`.
    pub storage: Arc<dyn StorageBackend>,
    /// Query engine
    pub query_engine: Arc<RwLock<QueryEngine>>,
    /// Trust manager
    pub trust_manager: Arc<RwLock<TrustManager>>,
    /// Application metrics
    pub metrics: Metrics,
    /// Server start time
    start_time: Instant,
}

impl AppState {
    /// Create new application state.
    ///
    /// `DESCI_STORAGE_BACKEND` selects the storage backend:
    /// - unset or `memory` (default): in-process `MemoryStorage`, ephemeral,
    ///   no external dependencies. Safe default -- whether
    ///   mycelix-knowledge is installed on any given conductor isn't
    ///   guaranteed, so a DHT-dependent default could silently break
    ///   startup for existing dev setups.
    /// - `knowledge_dht`: persists claims to mycelix-knowledge's DHT via
    ///   `KnowledgeDhtStorage` (native Holochain client, feature
    ///   `holochain`). Reads conductor URLs from `DESCI_HOLOCHAIN_ADMIN_URL`
    ///   / `DESCI_HOLOCHAIN_APP_URL` (default `ws://localhost:33800` /
    ///   `ws://localhost:8888`, matching the shared conductor's documented
    ///   ports) and `DESCI_HOLOCHAIN_APP_ID` (default `mycelix-knowledge`).
    pub async fn new() -> crate::error::Result<Self> {
        let backend =
            std::env::var("DESCI_STORAGE_BACKEND").unwrap_or_else(|_| "memory".to_string());

        let storage: Arc<dyn StorageBackend> = match backend.as_str() {
            "memory" => Arc::new(MemoryStorage::new()),
            #[cfg(feature = "holochain")]
            "knowledge_dht" => {
                let admin_url = std::env::var("DESCI_HOLOCHAIN_ADMIN_URL")
                    .unwrap_or_else(|_| "ws://localhost:33800".to_string());
                let app_url = std::env::var("DESCI_HOLOCHAIN_APP_URL")
                    .unwrap_or_else(|_| "ws://localhost:8888".to_string());
                let app_id = std::env::var("DESCI_HOLOCHAIN_APP_ID")
                    .unwrap_or_else(|_| "mycelix-knowledge".to_string());
                Arc::new(KnowledgeDhtStorage::connect(&admin_url, &app_url, &app_id).await?)
            }
            #[cfg(not(feature = "holochain"))]
            "knowledge_dht" => {
                return Err(mycelix_desci_core::Error::Storage(
                    "DESCI_STORAGE_BACKEND=knowledge_dht requires building with --features holochain"
                        .to_string(),
                )
                .into());
            }
            other => {
                return Err(mycelix_desci_core::Error::Storage(format!(
                    "Unknown DESCI_STORAGE_BACKEND: '{other}' (expected 'memory' or 'knowledge_dht')"
                ))
                .into());
            }
        };

        let query_engine = QueryEngine::new(storage.clone());
        let trust_manager = TrustManager::new();

        Ok(Self {
            storage,
            query_engine: Arc::new(RwLock::new(query_engine)),
            trust_manager: Arc::new(RwLock::new(trust_manager)),
            metrics: Metrics::new(),
            start_time: Instant::now(),
        })
    }

    /// Get uptime since server start
    pub fn uptime(&self) -> Duration {
        self.start_time.elapsed()
    }
}
