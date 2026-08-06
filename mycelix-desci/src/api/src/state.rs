// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Application state shared across handlers

use ed25519_dalek::{SigningKey, VerifyingKey};
use mycelix_desci_core::{
    AuthoritySigner, DefaultScientificAuthorizationPolicy, FileAuthorityWriteLeaseProvider,
    FileScientificAuthorityAuditStore, FileScientificEventLog, GovernedScientificEventLog,
    MemoryScientificAuthorityAuditStore, MemoryScientificEventLog, PostgresAuthorityConfig,
    PostgresAuthorityFencingConfig, PostgresAuthorityStore, ScientificAuthorityAuditStore,
    ScientificCredentialGovernance, ScientificCredentialRegistry, ScientificEventLog,
    query::QueryEngine,
    storage::{MemoryStorage, StorageBackend},
    trust::TrustManager,
};
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::{Duration, Instant};
use tokio::sync::RwLock;
use tracing::{info, warn};

#[cfg(unix)]
use mycelix_desci_core::UnixSocketAuthoritySigner;

const MAX_CREDENTIAL_BOOTSTRAP_TRUST_FILE_BYTES: u64 = 1024 * 1024;
const MAX_RECEIPT_TRUST_FILE_BYTES: u64 = 1024 * 1024;
const MAX_WRITE_LEASE_TRUST_FILE_BYTES: u64 = 1024 * 1024;

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
    /// Compatibility storage for pre-refoundation mutable claims. New
    /// authoritative writes must use `scientific_events`.
    pub storage: Arc<dyn StorageBackend>,
    /// Governed append-only scientific event log.
    pub scientific_events: Arc<dyn ScientificEventLog>,
    /// Independently persisted receipt-time authority evidence.
    pub scientific_authority_audit: Arc<dyn ScientificAuthorityAuditStore>,
    /// PostgreSQL atomic authority backend when selected. Exposed only to the
    /// durable outbox publisher and operational health reporting.
    pub postgres_authority_backend: Option<Arc<PostgresAuthorityStore>>,
    /// Append-only credential registry used for actor/key/role resolution.
    pub scientific_credentials: Arc<ScientificCredentialRegistry>,
    /// Threshold proposal and acceptance-service governance journal.
    pub credential_governance: Arc<ScientificCredentialGovernance>,
    /// Required number of active registry administrators before readiness passes.
    pub minimum_credential_registry_admins: usize,
    /// Required organization-diverse external witnesses on the latest published
    /// transparency checkpoint. Zero disables this deployment policy gate.
    pub minimum_checkpoint_witness_organizations: usize,
    /// Required independently operated mirror organizations that have persisted
    /// the latest governed checkpoint. Zero disables this deployment gate.
    pub minimum_checkpoint_mirror_organizations: usize,
    /// Maximum pending authority-outbox rows before readiness fails. Zero
    /// disables the backlog-count gate.
    pub maximum_pending_authority_outbox: i64,
    /// Maximum age in seconds of the oldest pending outbox row. Zero disables
    /// the age gate.
    pub maximum_authority_outbox_age_seconds: i64,
    /// Whether PostgreSQL deployments must publish at least one governed
    /// database epoch before readiness passes.
    pub require_authority_database_epoch: bool,
    /// Required organization-diverse acknowledgements of the latest published
    /// database-epoch delivery. Zero disables this policy gate.
    pub minimum_database_epoch_acknowledgement_organizations: usize,
    /// Whether the server has a configured key for signing authority receipts.
    pub authority_receipt_signing_configured: bool,
    /// Whether the selected PostgreSQL authority backend has a dedicated key
    /// for signing durable outbox delivery envelopes.
    pub authority_delivery_signing_configured: bool,
    /// Whether PostgreSQL authority writes must present an active externally
    /// signed lease bound to the governed primary and database epoch.
    pub require_authority_write_fencing: bool,
    /// Public key used to verify signed authority delivery envelopes.
    pub authority_delivery_public_key: Option<String>,
    /// Whether canonical events survive process restart. Memory mode is an
    /// explicit simulation and does not satisfy readiness.
    pub scientific_events_durable: bool,
    /// Explicit cutover override for pre-v0.5 events that have no receipt-time
    /// authority evidence. This never upgrades those events to attested status.
    pub allow_unattested_authority_history: bool,
    /// Compatibility mutation routes are disabled unless explicitly enabled.
    pub legacy_mutations_enabled: bool,
    /// Query engine for legacy compatibility records.
    pub query_engine: Arc<RwLock<QueryEngine>>,
    /// Trust manager retained for compatibility reads only.
    pub trust_manager: Arc<RwLock<TrustManager>>,
    /// Application metrics
    pub metrics: Metrics,
    /// Server start time
    start_time: Instant,
}

impl AppState {
    /// Create new application state.
    ///
    /// Canonical authority configuration:
    ///
    /// - `DESCI_CREDENTIAL_BOOTSTRAP_TRUST_FILE` contains only the public keys
    ///   trusted to initialize the append-only credential registry.
    /// - `DESCI_CREDENTIAL_REGISTRY_PATH` stores signed actor/key/role events.
    /// - `DESCI_CREDENTIAL_GOVERNANCE_PATH` stores delayed threshold proposals,
    ///   approvals, executions, service-key changes, and transparency checkpoints.
    /// - `DESCI_AUTHORITY_RECEIPT_SIGNING_KEY_FILE` contains the current
    ///   server Ed25519 service seed (32 raw bytes or 64 hex chars). Domain-
    ///   separated signatures cover both credential acceptance timestamps and
    ///   scientific authority receipts.
    /// - `DESCI_AUTHORITY_RECEIPT_TRUST_FILE` optionally contains a JSON array
    ///   of historical trusted receipt public keys as 64-character hex strings.
    /// - `DESCI_SCIENTIFIC_AUTHORITY_AUDIT_PATH` defaults to
    ///   `./data/scientific-authority`.
    /// - `DESCI_ALLOW_UNATTESTED_AUTHORITY_HISTORY=true` is an explicit
    ///   temporary cutover acknowledgement for canonical events written before
    ///   receipt journaling existed. Those events remain visibly unattested.
    pub async fn new() -> crate::error::Result<Self> {
        let backend =
            std::env::var("DESCI_STORAGE_BACKEND").unwrap_or_else(|_| "memory".to_string());

        let storage: Arc<dyn StorageBackend> = match backend.as_str() {
            "memory" => Arc::new(MemoryStorage::new()),
            "knowledge_dht" => {
                // `KnowledgeDhtStorage` moved to the standalone
                // mycelix-desci-holochain-bridge crate/workspace 2026-08-05
                // (its holochain_types -> rusqlite dependency is
                // unconditionally incompatible, at the Cargo.lock level, with
                // this crate's sqlx-postgres dependency -- see that crate's
                // module doc comment). It is not linked into this binary and
                // would need to run as a separate process/service to be used
                // again.
                return Err(mycelix_desci_core::Error::Storage(
                    "DESCI_STORAGE_BACKEND=knowledge_dht is not available in this build -- \
                     the Holochain DHT storage backend was split into the standalone \
                     mycelix-desci-holochain-bridge crate and is not linked into mycelix-api"
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

        let bootstrap_trust_keys = load_credential_bootstrap_trust_keys()?;
        let receipt_signing_key = load_receipt_signing_key()?.map(Arc::new);
        let authority_receipt_signing_configured = receipt_signing_key.is_some();
        if !authority_receipt_signing_configured {
            warn!(
                "no DESCI_AUTHORITY_RECEIPT_SIGNING_KEY_FILE configured; canonical writes and credential mutations will fail closed"
            );
        }
        let mut trusted_receipt_keys = load_trusted_receipt_keys()?;
        if let Some(signing_key) = &receipt_signing_key {
            trusted_receipt_keys.insert(signing_key.verifying_key().to_bytes());
        }
        let delivery_signer = load_authority_delivery_signer()?;
        let authority_delivery_signing_configured = delivery_signer.is_some();
        let authority_delivery_public_key = delivery_signer
            .as_ref()
            .map(|signer| hex::encode(signer.verifying_key().to_bytes()));

        let scientific_backend =
            std::env::var("DESCI_SCIENTIFIC_EVENT_BACKEND").unwrap_or_else(|_| "file".to_string());
        let (raw_scientific_events, scientific_events_durable, postgres_authority): (
            Arc<dyn ScientificEventLog>,
            bool,
            Option<Arc<PostgresAuthorityStore>>,
        ) = match scientific_backend.as_str() {
            "file" => {
                let path = std::env::var("DESCI_SCIENTIFIC_EVENT_PATH")
                    .map(PathBuf::from)
                    .unwrap_or_else(|_| PathBuf::from("./data/scientific-events"));
                info!(path = %path.display(), "opening durable scientific event log");
                (
                    Arc::new(FileScientificEventLog::open(path).await?),
                    true,
                    None,
                )
            }
            "postgres" => {
                let database_url = std::env::var("DESCI_POSTGRES_URL").map_err(|_| {
                    mycelix_desci_core::Error::Validation(
                        "DESCI_POSTGRES_URL is required when DESCI_SCIENTIFIC_EVENT_BACKEND=postgres"
                            .to_string(),
                    )
                })?;
                let mut config = PostgresAuthorityConfig::from_database_url(database_url);
                if let Ok(value) = std::env::var("DESCI_POSTGRES_MAX_CONNECTIONS") {
                    config.max_connections = value.parse::<u32>().map_err(|error| {
                        mycelix_desci_core::Error::Validation(format!(
                            "invalid DESCI_POSTGRES_MAX_CONNECTIONS: {error}"
                        ))
                    })?;
                }
                if let Ok(value) = std::env::var("DESCI_POSTGRES_ACQUIRE_TIMEOUT_SECONDS") {
                    let seconds = value.parse::<u64>().map_err(|error| {
                        mycelix_desci_core::Error::Validation(format!(
                            "invalid DESCI_POSTGRES_ACQUIRE_TIMEOUT_SECONDS: {error}"
                        ))
                    })?;
                    config.acquire_timeout = Duration::from_secs(seconds);
                }
                config.run_migrations =
                    env_bool_with_default("DESCI_POSTGRES_AUTO_MIGRATE", false)?;
                info!(
                    auto_migrate = config.run_migrations,
                    "opening PostgreSQL atomic scientific authority backend"
                );
                let delivery_signer = delivery_signer.clone().ok_or_else(|| {
                    mycelix_desci_core::Error::Validation(
                        "an authority outbox signer is required when DESCI_SCIENTIFIC_EVENT_BACKEND=postgres"
                            .to_string(),
                    )
                })?;
                let require_write_fencing =
                    env_bool_with_default("DESCI_REQUIRE_AUTHORITY_WRITE_FENCING", true)?;
                let fencing = load_postgres_authority_fencing_config(require_write_fencing)?;
                let store = Arc::new(match fencing {
                    Some(fencing) => {
                        PostgresAuthorityStore::connect_with_fencing(
                            config,
                            trusted_receipt_keys.clone(),
                            delivery_signer,
                            fencing,
                        )
                        .await?
                    }
                    None => {
                        PostgresAuthorityStore::connect(
                            config,
                            trusted_receipt_keys.clone(),
                            delivery_signer,
                        )
                        .await?
                    }
                });
                (store.clone(), true, Some(store))
            }
            "memory" => {
                warn!(
                    "DESCI_SCIENTIFIC_EVENT_BACKEND=memory: canonical events are ephemeral simulation state"
                );
                (Arc::new(MemoryScientificEventLog::new()), false, None)
            }
            other => {
                return Err(mycelix_desci_core::Error::Storage(format!(
                    "Unknown DESCI_SCIENTIFIC_EVENT_BACKEND: '{other}' (expected 'file', 'postgres', or 'memory')"
                ))
                .into());
            }
        };

        let credential_backend =
            std::env::var("DESCI_CREDENTIAL_REGISTRY_BACKEND").unwrap_or_else(|_| {
                match scientific_backend.as_str() {
                    "postgres" => "postgres".to_string(),
                    "memory" => "memory".to_string(),
                    _ => "file".to_string(),
                }
            });
        let scientific_credentials = Arc::new(match credential_backend.as_str() {
            "file" => {
                let path = std::env::var("DESCI_CREDENTIAL_REGISTRY_PATH")
                    .map(PathBuf::from)
                    .unwrap_or_else(|_| PathBuf::from("./data/scientific-credentials.json"));
                info!(path = %path.display(), "opening scientific credential registry");
                ScientificCredentialRegistry::open_file(
                    path,
                    bootstrap_trust_keys,
                    trusted_receipt_keys.clone(),
                    receipt_signing_key.clone(),
                )
                .await?
            }
            "postgres" => {
                let store = postgres_authority.clone().ok_or_else(|| {
                    mycelix_desci_core::Error::Storage(
                        "DESCI_CREDENTIAL_REGISTRY_BACKEND=postgres requires DESCI_SCIENTIFIC_EVENT_BACKEND=postgres"
                            .to_string(),
                    )
                })?;
                info!("opening PostgreSQL scientific credential registry");
                ScientificCredentialRegistry::open_postgres(
                    store,
                    bootstrap_trust_keys,
                    trusted_receipt_keys.clone(),
                    receipt_signing_key.clone(),
                )
                .await?
            }
            "memory" => {
                warn!(
                    "DESCI_CREDENTIAL_REGISTRY_BACKEND=memory: actor authority is ephemeral simulation state"
                );
                ScientificCredentialRegistry::memory(
                    bootstrap_trust_keys,
                    trusted_receipt_keys.clone(),
                    receipt_signing_key.clone(),
                )
            }
            other => {
                return Err(mycelix_desci_core::Error::Storage(format!(
                    "Unknown DESCI_CREDENTIAL_REGISTRY_BACKEND: '{other}' (expected 'file', 'postgres', or 'memory')"
                ))
                .into());
            }
        });
        if let Some(signer) = &delivery_signer {
            if scientific_credentials
                .projection()
                .await
                .owns_signing_key(&signer.verifying_key().to_bytes())
            {
                return Err(mycelix_desci_core::Error::VerificationFailed(
                    "authority outbox signing key must not be registered as a scientific actor key"
                        .to_string(),
                )
                .into());
            }
        }
        if let Some(store) = postgres_authority.as_ref() {
            let projection = scientific_credentials.projection().await;
            if store
                .trusted_write_lease_keys()
                .iter()
                .any(|key| projection.owns_signing_key(key))
            {
                return Err(mycelix_desci_core::Error::VerificationFailed(
                    "authority write-lease keys must not be registered as scientific actor keys"
                        .to_string(),
                )
                .into());
            }
        }
        let credential_summary = scientific_credentials.summary().await;
        if !credential_summary.initialized {
            warn!(
                "scientific credential registry is uninitialized; canonical writes will fail closed"
            );
        }
        let governance_backend = std::env::var("DESCI_CREDENTIAL_GOVERNANCE_BACKEND")
            .unwrap_or_else(|_| credential_backend.clone());
        let credential_governance = match governance_backend.as_str() {
            "file" => {
                let path = std::env::var("DESCI_CREDENTIAL_GOVERNANCE_PATH")
                    .map(PathBuf::from)
                    .unwrap_or_else(|_| {
                        PathBuf::from("./data/scientific-credential-governance.json")
                    });
                info!(path = %path.display(), "opening threshold credential governance journal");
                ScientificCredentialGovernance::open_file(
                    scientific_credentials.clone(),
                    path,
                    trusted_receipt_keys.clone(),
                    receipt_signing_key.clone(),
                )
                .await?
            }
            "postgres" => {
                let store = postgres_authority.clone().ok_or_else(|| {
                    mycelix_desci_core::Error::Storage(
                        "DESCI_CREDENTIAL_GOVERNANCE_BACKEND=postgres requires DESCI_SCIENTIFIC_EVENT_BACKEND=postgres"
                            .to_string(),
                    )
                })?;
                info!("opening PostgreSQL threshold credential governance journal");
                ScientificCredentialGovernance::open_postgres(
                    scientific_credentials.clone(),
                    store,
                    trusted_receipt_keys.clone(),
                    receipt_signing_key.clone(),
                )
                .await?
            }
            "memory" => {
                warn!(
                    "DESCI_CREDENTIAL_GOVERNANCE_BACKEND=memory: threshold authority is ephemeral simulation state"
                );
                ScientificCredentialGovernance::memory(
                    scientific_credentials.clone(),
                    trusted_receipt_keys.clone(),
                    receipt_signing_key.clone(),
                )
                .await?
            }
            other => {
                return Err(mycelix_desci_core::Error::Storage(format!(
                    "Unknown DESCI_CREDENTIAL_GOVERNANCE_BACKEND: '{other}' (expected 'file', 'postgres', or 'memory')"
                ))
                .into());
            }
        };
        if scientific_backend == "postgres"
            && (credential_backend != "postgres" || governance_backend != "postgres")
        {
            return Err(mycelix_desci_core::Error::Validation(
                "PostgreSQL scientific authority requires both DESCI_CREDENTIAL_REGISTRY_BACKEND=postgres and DESCI_CREDENTIAL_GOVERNANCE_BACKEND=postgres to avoid split-brain authority state"
                    .to_string(),
            )
            .into());
        }
        let governance_summary = credential_governance.summary().await;
        if !governance_summary.initialized {
            warn!(
                "threshold credential governance is uninitialized; readiness and governed credential execution remain disabled"
            );
        }
        let credential_governance = Arc::new(credential_governance);

        let minimum_credential_registry_admins =
            std::env::var("DESCI_MIN_CREDENTIAL_REGISTRY_ADMINS")
                .ok()
                .and_then(|value| value.parse::<usize>().ok())
                .filter(|value| (1..=32).contains(value))
                .unwrap_or(2);

        let minimum_checkpoint_witness_organizations =
            std::env::var("DESCI_MIN_CHECKPOINT_WITNESS_ORGANIZATIONS")
                .ok()
                .and_then(|value| value.parse::<usize>().ok())
                .filter(|value| *value <= 32)
                .unwrap_or(0);
        let minimum_checkpoint_mirror_organizations =
            std::env::var("DESCI_MIN_CHECKPOINT_MIRROR_ORGANIZATIONS")
                .ok()
                .and_then(|value| value.parse::<usize>().ok())
                .filter(|value| *value <= 32)
                .unwrap_or(0);
        let maximum_pending_authority_outbox = std::env::var("DESCI_MAX_PENDING_AUTHORITY_OUTBOX")
            .ok()
            .and_then(|value| value.parse::<i64>().ok())
            .filter(|value| (0..=10_000_000).contains(value))
            .unwrap_or(0);
        let maximum_authority_outbox_age_seconds =
            std::env::var("DESCI_MAX_AUTHORITY_OUTBOX_AGE_SECONDS")
                .ok()
                .and_then(|value| value.parse::<i64>().ok())
                .filter(|value| (0..=31_536_000).contains(value))
                .unwrap_or(0);
        let require_authority_database_epoch = env_bool_with_default(
            "DESCI_REQUIRE_AUTHORITY_DATABASE_EPOCH",
            scientific_backend == "postgres",
        )?;
        let minimum_database_epoch_acknowledgement_organizations =
            std::env::var("DESCI_MIN_DATABASE_EPOCH_ACK_ORGANIZATIONS")
                .ok()
                .and_then(|value| value.parse::<usize>().ok())
                .filter(|value| *value <= 32)
                .unwrap_or(0);

        let scientific_authority_audit: Arc<dyn ScientificAuthorityAuditStore> =
            match scientific_backend.as_str() {
                "file" => {
                    let path = std::env::var("DESCI_SCIENTIFIC_AUTHORITY_AUDIT_PATH")
                        .map(PathBuf::from)
                        .unwrap_or_else(|_| PathBuf::from("./data/scientific-authority"));
                    info!(path = %path.display(), "opening scientific authority audit journal");
                    Arc::new(
                        FileScientificAuthorityAuditStore::open(path, trusted_receipt_keys.clone())
                            .await?,
                    )
                }
                "postgres" => postgres_authority.as_ref().cloned().ok_or_else(|| {
                    mycelix_desci_core::Error::Storage(
                        "PostgreSQL authority backend was not initialized".to_string(),
                    )
                })?,
                "memory" => Arc::new(MemoryScientificAuthorityAuditStore::new(
                    trusted_receipt_keys.clone(),
                )),
                _ => unreachable!("scientific backend validated above"),
            };
        let audit_summary = scientific_authority_audit
            .reconcile(raw_scientific_events.as_ref())
            .await?;

        let allow_unattested_authority_history =
            env_flag("DESCI_ALLOW_UNATTESTED_AUTHORITY_HISTORY");
        if audit_summary.legacy_unattested_events > 0 {
            warn!(
                legacy_unattested_events = audit_summary.legacy_unattested_events,
                allow_unattested_authority_history,
                "canonical history contains a pre-receipt prefix; it remains explicitly unattested"
            );
        }
        if audit_summary.unsafe_unattested_events > 0 {
            warn!(
                unsafe_unattested_events = audit_summary.unsafe_unattested_events,
                "canonical history contains missing authority receipts after receipt journaling began; readiness is blocked"
            );
        }

        let governed = GovernedScientificEventLog::new(
            raw_scientific_events,
            scientific_credentials.clone(),
            DefaultScientificAuthorizationPolicy,
            scientific_authority_audit.clone(),
            receipt_signing_key,
        );
        let governed = if let Some(store) = postgres_authority.as_ref() {
            governed.with_atomic_committer(store.clone())
        } else {
            governed
        };
        let scientific_events: Arc<dyn ScientificEventLog> = Arc::new(governed);

        let require_authority_write_fencing = env_bool_with_default(
            "DESCI_REQUIRE_AUTHORITY_WRITE_FENCING",
            scientific_backend == "postgres",
        )?;
        let legacy_mutations_enabled = env_flag("DESCI_ENABLE_LEGACY_MUTATIONS");
        if legacy_mutations_enabled {
            warn!(
                "legacy mutable claim routes are enabled; these writes are not canonical scientific events"
            );
        }

        let query_engine = QueryEngine::new(storage.clone());
        let trust_manager = TrustManager::new();

        Ok(Self {
            storage,
            scientific_events,
            scientific_authority_audit,
            postgres_authority_backend: postgres_authority,
            scientific_credentials,
            credential_governance,
            minimum_credential_registry_admins,
            minimum_checkpoint_witness_organizations,
            minimum_checkpoint_mirror_organizations,
            maximum_pending_authority_outbox,
            maximum_authority_outbox_age_seconds,
            require_authority_database_epoch,
            minimum_database_epoch_acknowledgement_organizations,
            authority_receipt_signing_configured,
            authority_delivery_signing_configured,
            require_authority_write_fencing,
            authority_delivery_public_key,
            scientific_events_durable,
            allow_unattested_authority_history,
            legacy_mutations_enabled,
            query_engine: Arc::new(RwLock::new(query_engine)),
            trust_manager: Arc::new(RwLock::new(trust_manager)),
            metrics: Metrics::new(),
            start_time: Instant::now(),
        })
    }

    /// Refresh credential and governance projections from their durable
    /// backend before serving authority-sensitive reads.
    pub async fn synchronize_authority_state(&self) -> crate::error::Result<()> {
        self.scientific_credentials.synchronize().await?;
        self.credential_governance.synchronize().await?;
        Ok(())
    }

    /// Get uptime since server start
    pub fn uptime(&self) -> Duration {
        self.start_time.elapsed()
    }
}

fn load_credential_bootstrap_trust_keys() -> crate::error::Result<BTreeSet<[u8; 32]>> {
    let path = match std::env::var("DESCI_CREDENTIAL_BOOTSTRAP_TRUST_FILE") {
        Ok(path) if !path.trim().is_empty() => PathBuf::from(path),
        _ => return Ok(BTreeSet::new()),
    };
    let bytes = read_regular_file(
        &path,
        MAX_CREDENTIAL_BOOTSTRAP_TRUST_FILE_BYTES,
        "credential bootstrap trust file",
        false,
    )?;
    let encoded: Vec<String> = serde_json::from_slice(&bytes).map_err(|error| {
        crate::error::ApiError::invalid_request(format!(
            "invalid credential bootstrap trust file {}: {error}",
            path.display()
        ))
    })?;
    if encoded.is_empty() {
        return Err(crate::error::ApiError::invalid_request(
            "credential bootstrap trust file cannot be empty",
        ));
    }
    decode_public_key_set(encoded, &path, "credential bootstrap")
}

fn load_receipt_signing_key() -> crate::error::Result<Option<SigningKey>> {
    let path = match std::env::var("DESCI_AUTHORITY_RECEIPT_SIGNING_KEY_FILE") {
        Ok(path) if !path.trim().is_empty() => PathBuf::from(path),
        _ => return Ok(None),
    };
    let bytes = read_regular_file(&path, 4096, "authority receipt signing key", true)?;
    let key = decode_signing_key(&bytes, "authority receipt signing key")?;
    info!(
        path = %path.display(),
        public_key = %hex::encode(key.verifying_key().to_bytes()),
        "loaded authority receipt signing key"
    );
    Ok(Some(key))
}

fn load_authority_delivery_signer() -> crate::error::Result<Option<Arc<dyn AuthoritySigner>>> {
    if let Ok(socket) = std::env::var("DESCI_AUTHORITY_OUTBOX_SIGNER_SOCKET") {
        if !socket.trim().is_empty() {
            #[cfg(not(unix))]
            {
                return Err(crate::error::ApiError::invalid_request(
                    "DESCI_AUTHORITY_OUTBOX_SIGNER_SOCKET is supported only on Unix platforms",
                ));
            }
            #[cfg(unix)]
            {
                let key_id = required_env("DESCI_AUTHORITY_OUTBOX_SIGNER_KEY_ID")?;
                let public_key = decode_verifying_key_hex(
                    &required_env("DESCI_AUTHORITY_OUTBOX_SIGNER_PUBLIC_KEY")?,
                    "authority outbox signer public key",
                )?;
                let timeout_millis = std::env::var("DESCI_AUTHORITY_OUTBOX_SIGNER_TIMEOUT_MILLIS")
                    .ok()
                    .and_then(|value| value.parse::<u64>().ok())
                    .filter(|value| (1..=60_000).contains(value))
                    .unwrap_or(5_000);
                let signer = UnixSocketAuthoritySigner::new(
                    key_id,
                    public_key,
                    PathBuf::from(socket),
                    Duration::from_millis(timeout_millis),
                )?;
                info!(
                    key_id = signer.key_id(),
                    public_key = %hex::encode(signer.verifying_key().to_bytes()),
                    "configured Unix-socket authority outbox signer"
                );
                return Ok(Some(Arc::new(signer)));
            }
        }
    }

    let path = match std::env::var("DESCI_AUTHORITY_OUTBOX_SIGNING_KEY_FILE") {
        Ok(path) if !path.trim().is_empty() => PathBuf::from(path),
        _ => return Ok(None),
    };
    let bytes = read_regular_file(&path, 4096, "authority outbox signing key", true)?;
    let key = decode_signing_key(&bytes, "authority outbox signing key")?;
    info!(
        path = %path.display(),
        public_key = %hex::encode(key.verifying_key().to_bytes()),
        "loaded authority outbox signing key"
    );
    Ok(Some(Arc::new(key)))
}

fn load_postgres_authority_fencing_config(
    required: bool,
) -> crate::error::Result<Option<PostgresAuthorityFencingConfig>> {
    let deployment = std::env::var("DESCI_AUTHORITY_DEPLOYMENT_ID").ok();
    let lease_file = std::env::var("DESCI_AUTHORITY_WRITE_LEASE_FILE").ok();
    let trust_file = std::env::var("DESCI_AUTHORITY_WRITE_LEASE_TRUST_FILE").ok();
    let any_configured = deployment
        .as_ref()
        .is_some_and(|value| !value.trim().is_empty())
        || lease_file
            .as_ref()
            .is_some_and(|value| !value.trim().is_empty())
        || trust_file
            .as_ref()
            .is_some_and(|value| !value.trim().is_empty());
    if !required && !any_configured {
        return Ok(None);
    }
    let deployment = deployment
        .filter(|value| !value.trim().is_empty())
        .ok_or_else(|| {
            crate::error::ApiError::invalid_request(
                "DESCI_AUTHORITY_DEPLOYMENT_ID is required for PostgreSQL write fencing",
            )
        })?;
    let lease_file = lease_file
        .filter(|value| !value.trim().is_empty())
        .map(PathBuf::from)
        .ok_or_else(|| {
            crate::error::ApiError::invalid_request(
                "DESCI_AUTHORITY_WRITE_LEASE_FILE is required for PostgreSQL write fencing",
            )
        })?;
    let trust_file = trust_file
        .filter(|value| !value.trim().is_empty())
        .map(PathBuf::from)
        .ok_or_else(|| {
            crate::error::ApiError::invalid_request(
                "DESCI_AUTHORITY_WRITE_LEASE_TRUST_FILE is required for PostgreSQL write fencing",
            )
        })?;
    let trusted = load_public_key_trust_file(
        &trust_file,
        MAX_WRITE_LEASE_TRUST_FILE_BYTES,
        "authority write-lease trust file",
    )?;
    let provider = Arc::new(FileAuthorityWriteLeaseProvider::new(lease_file)?);
    let mut config = PostgresAuthorityFencingConfig::new(deployment, trusted, provider);
    let skew_seconds = std::env::var("DESCI_AUTHORITY_WRITE_LEASE_CLOCK_SKEW_SECONDS")
        .ok()
        .and_then(|value| value.parse::<i64>().ok())
        .filter(|value| (0..=300).contains(value))
        .unwrap_or(30);
    config.clock_skew = chrono::Duration::seconds(skew_seconds);
    config.validate()?;
    Ok(Some(config))
}

fn required_env(name: &str) -> crate::error::Result<String> {
    std::env::var(name)
        .ok()
        .filter(|value| !value.trim().is_empty())
        .map(|value| value.trim().to_string())
        .ok_or_else(|| crate::error::ApiError::invalid_request(format!("{name} is required")))
}

fn decode_verifying_key_hex(value: &str, label: &str) -> crate::error::Result<VerifyingKey> {
    let decoded = hex::decode(value.trim()).map_err(|error| {
        crate::error::ApiError::invalid_request(format!("invalid {label}: {error}"))
    })?;
    let bytes: [u8; 32] = decoded.try_into().map_err(|_| {
        crate::error::ApiError::invalid_request(format!("{label} must decode to exactly 32 bytes"))
    })?;
    VerifyingKey::from_bytes(&bytes).map_err(|error| {
        crate::error::ApiError::invalid_request(format!("invalid {label}: {error}"))
    })
}

fn load_public_key_trust_file(
    path: &Path,
    max_bytes: u64,
    label: &str,
) -> crate::error::Result<BTreeSet<[u8; 32]>> {
    let bytes = read_regular_file(path, max_bytes, label, false)?;
    let encoded: Vec<String> = serde_json::from_slice(&bytes).map_err(|error| {
        crate::error::ApiError::invalid_request(format!(
            "invalid {label} {}: {error}",
            path.display()
        ))
    })?;
    if encoded.is_empty() {
        return Err(crate::error::ApiError::invalid_request(format!(
            "{label} cannot be empty"
        )));
    }
    decode_public_key_set(encoded, path, label)
}

fn load_trusted_receipt_keys() -> crate::error::Result<BTreeSet<[u8; 32]>> {
    let path = match std::env::var("DESCI_AUTHORITY_RECEIPT_TRUST_FILE") {
        Ok(path) if !path.trim().is_empty() => PathBuf::from(path),
        _ => return Ok(BTreeSet::new()),
    };
    let bytes = read_regular_file(
        &path,
        MAX_RECEIPT_TRUST_FILE_BYTES,
        "authority receipt trust file",
        false,
    )?;
    let encoded: Vec<String> = serde_json::from_slice(&bytes).map_err(|error| {
        crate::error::ApiError::invalid_request(format!(
            "invalid authority receipt trust file {}: {error}",
            path.display()
        ))
    })?;
    let mut keys = BTreeSet::new();
    for value in encoded {
        let decoded = hex::decode(value.trim()).map_err(|error| {
            crate::error::ApiError::invalid_request(format!(
                "invalid authority receipt public key in {}: {error}",
                path.display()
            ))
        })?;
        let key: [u8; 32] = decoded.try_into().map_err(|_| {
            crate::error::ApiError::invalid_request(format!(
                "authority receipt public keys in {} must decode to 32 bytes",
                path.display()
            ))
        })?;
        if !keys.insert(key) {
            return Err(crate::error::ApiError::invalid_request(format!(
                "duplicate authority receipt public key in {}",
                path.display()
            )));
        }
    }
    Ok(keys)
}

fn decode_public_key_set(
    encoded: Vec<String>,
    path: &Path,
    label: &str,
) -> crate::error::Result<BTreeSet<[u8; 32]>> {
    let mut keys = BTreeSet::new();
    for value in encoded {
        let decoded = hex::decode(value.trim()).map_err(|error| {
            crate::error::ApiError::invalid_request(format!(
                "invalid {label} public key in {}: {error}",
                path.display()
            ))
        })?;
        let key: [u8; 32] = decoded.try_into().map_err(|_| {
            crate::error::ApiError::invalid_request(format!(
                "{label} public keys in {} must decode to 32 bytes",
                path.display()
            ))
        })?;
        if !keys.insert(key) {
            return Err(crate::error::ApiError::invalid_request(format!(
                "duplicate {label} public key in {}",
                path.display()
            )));
        }
    }
    Ok(keys)
}

fn decode_signing_key(bytes: &[u8], label: &str) -> crate::error::Result<SigningKey> {
    let decoded = if bytes.len() == 32 {
        bytes.to_vec()
    } else {
        let text = std::str::from_utf8(bytes).map_err(|_| {
            crate::error::ApiError::invalid_request(format!(
                "{label} must be 32 raw bytes or 64 hexadecimal characters"
            ))
        })?;
        hex::decode(text.trim()).map_err(|error| {
            crate::error::ApiError::invalid_request(format!("invalid hexadecimal {label}: {error}"))
        })?
    };
    let key: [u8; 32] = decoded.try_into().map_err(|_| {
        crate::error::ApiError::invalid_request(format!("{label} must decode to exactly 32 bytes"))
    })?;
    Ok(SigningKey::from_bytes(&key))
}

fn read_regular_file(
    path: &Path,
    max_bytes: u64,
    label: &str,
    require_private_permissions: bool,
) -> crate::error::Result<Vec<u8>> {
    let metadata = std::fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() {
        return Err(crate::error::ApiError::invalid_request(format!(
            "{label} must not be a symbolic link: {}",
            path.display()
        )));
    }
    if !metadata.is_file() {
        return Err(crate::error::ApiError::invalid_request(format!(
            "{label} must be a regular file: {}",
            path.display()
        )));
    }
    if metadata.len() > max_bytes {
        return Err(crate::error::ApiError::invalid_request(format!(
            "{label} exceeds {max_bytes} bytes: {}",
            path.display()
        )));
    }
    #[cfg(unix)]
    if require_private_permissions {
        use std::os::unix::fs::PermissionsExt;
        if metadata.permissions().mode() & 0o077 != 0 {
            return Err(crate::error::ApiError::invalid_request(format!(
                "{label} permissions are too broad; require mode 0600 or stricter"
            )));
        }
    }
    Ok(std::fs::read(path)?)
}

fn env_bool_with_default(name: &str, default: bool) -> crate::error::Result<bool> {
    let Ok(value) = std::env::var(name) else {
        return Ok(default);
    };
    match value.trim().to_ascii_lowercase().as_str() {
        "1" | "true" | "yes" => Ok(true),
        "0" | "false" | "no" => Ok(false),
        _ => Err(crate::error::ApiError::invalid_request(format!(
            "{name} must be true or false"
        ))),
    }
}

fn env_flag(name: &str) -> bool {
    std::env::var(name)
        .ok()
        .is_some_and(|value| value.eq_ignore_ascii_case("true"))
}
