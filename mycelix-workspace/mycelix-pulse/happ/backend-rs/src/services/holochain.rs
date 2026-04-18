// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Holochain client service
//!
//! Provides typed access to Mycelix-Mail DNA zome functions.
//! All DID resolution happens via DHT - no external registry needed.
//!
//! Now integrated with mycelix-identity hApp for:
//! - Real DID resolution from identity DNA
//! - Credential verification before message send
//! - Assurance level checks (E0-E4)
//!
//! Supports two modes:
//! - **Production mode**: Connects to a real Holochain conductor via WebSocket
//! - **Stub mode**: Returns mock data for testing (HOLOCHAIN_STUB_MODE=true)
//!
//! Graceful degradation: If identity hApp is unavailable, operations continue
//! with warnings rather than failures.

use std::collections::HashMap;
use std::fmt::Debug;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::RwLock;
use tokio::time::timeout;
use holochain_client::{
    AdminWebsocket, AppWebsocket, AgentSigner, ClientAgentSigner,
    CellInfo, ExternIO, AppInfo,
    IssueAppAuthenticationTokenPayload,
};
use holochain_types::prelude::{AgentPubKey, ActionHash, CellId};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

use crate::config::Config;
use crate::error::{AppError, AppResult};

/// Connection state for the Holochain service
#[derive(Debug)]
enum ConnectionState {
    /// Not connected to conductor
    Disconnected,
    /// Connected in stub mode (no real conductor)
    StubMode {
        mock_agent_pubkey: AgentPubKey,
    },
    /// Connected to real conductor
    Connected {
        app_ws: AppWebsocket,
        agent_pubkey: AgentPubKey,
        cell_id: CellId,
    },
}

/// Cache entry for DID documents
struct DidCacheEntry {
    document: DidDocument,
    expires_at: Instant,
}

/// Holochain client wrapper with connection management
///
/// This service handles:
/// - WebSocket connection to Holochain conductor
/// - Agent authentication via signing credentials
/// - Automatic reconnection on failure
/// - Stub mode for testing without a conductor
/// - Identity hApp integration for DID resolution and credential verification
pub struct HolochainService {
    config: Config,
    state: Arc<RwLock<ConnectionState>>,
    reconnect_attempts: Arc<RwLock<u32>>,
    /// Identity hApp cell ID (if available)
    identity_cell_id: Arc<RwLock<Option<CellId>>>,
    /// Whether identity hApp is available
    identity_happ_available: Arc<RwLock<bool>>,
    /// DID document cache
    did_cache: Arc<RwLock<HashMap<String, DidCacheEntry>>>,
    /// Cache TTL (5 minutes default)
    cache_ttl: Duration,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct DidBinding {
    pub did: String,
    pub agent_pub_key: AgentPubKey,
}

impl HolochainService {
    /// Create a new Holochain service
    pub fn new(config: Config) -> Self {
        Self {
            config,
            state: Arc::new(RwLock::new(ConnectionState::Disconnected)),
            reconnect_attempts: Arc::new(RwLock::new(0)),
            identity_cell_id: Arc::new(RwLock::new(None)),
            identity_happ_available: Arc::new(RwLock::new(false)),
            did_cache: Arc::new(RwLock::new(HashMap::new())),
            cache_ttl: Duration::from_secs(300), // 5 minutes
        }
    }

    /// Connect to the Holochain conductor
    ///
    /// In stub mode (HOLOCHAIN_STUB_MODE=true), this creates a mock connection.
    /// In production mode, this connects to the actual conductor.
    pub async fn connect(&self) -> AppResult<()> {
        let mut state = self.state.write().await;

        // Check if already connected
        match &*state {
            ConnectionState::Connected { .. } | ConnectionState::StubMode { .. } => {
                return Ok(());
            }
            ConnectionState::Disconnected => {}
        }

        // Check for stub mode
        if self.config.is_stub_mode() {
            tracing::info!("Initializing Holochain service in STUB MODE");
            // Create a valid mock agent pubkey with proper prefix
            // AgentPubKey has prefix [0x84, 0x20, 0x24]
            let mock_agent_pubkey = Self::mock_agent_pubkey();
            *state = ConnectionState::StubMode { mock_agent_pubkey };
            tracing::info!("Stub mode initialized successfully");
            return Ok(());
        }

        // Production mode: connect to real conductor
        tracing::info!(
            "Connecting to Holochain conductor at {}",
            self.config.holochain_conductor_url
        );

        // Connect with timeout
        let connect_timeout = self.config.holochain_connect_timeout();

        match timeout(connect_timeout, self.establish_connection()).await {
            Ok(Ok((app_ws, agent_pubkey, cell_id))) => {
                *state = ConnectionState::Connected {
                    app_ws,
                    agent_pubkey: agent_pubkey.clone(),
                    cell_id,
                };
                *self.reconnect_attempts.write().await = 0;
                tracing::info!(
                    "Connected to Holochain conductor, agent: {:?}",
                    agent_pubkey
                );
                Ok(())
            }
            Ok(Err(e)) => {
                tracing::error!("Failed to connect to Holochain conductor: {}", e);
                Err(e)
            }
            Err(_) => {
                tracing::error!(
                    "Connection to Holochain conductor timed out after {:?}",
                    connect_timeout
                );
                Err(AppError::HolochainError(format!(
                    "Connection timed out after {:?}",
                    connect_timeout
                )))
            }
        }
    }

    /// Establish connection to the conductor (internal helper)
    async fn establish_connection(&self) -> AppResult<(AppWebsocket, AgentPubKey, CellId)> {
        // Step 1: Connect to admin interface to get authentication token
        tracing::debug!("Connecting to admin interface at {}", self.config.holochain_admin_url);

        let admin_ws = AdminWebsocket::connect(&self.config.holochain_admin_url)
            .await
            .map_err(|e| AppError::HolochainError(format!("Admin connection failed: {}", e)))?;

        // Step 2: Get app info to find the cell
        let installed_app_id = self.config.holochain_app_id.clone();

        // Issue an authentication token for the app
        let token_payload = IssueAppAuthenticationTokenPayload {
            installed_app_id: installed_app_id.clone().into(),
            expiry_seconds: 3600, // 1 hour token
            single_use: false,
        };

        let token_response = admin_ws
            .issue_app_auth_token(token_payload)
            .await
            .map_err(|e| AppError::HolochainError(format!("Failed to issue auth token: {}", e)))?;

        // Step 3: Create a ClientAgentSigner for signing zome calls
        let signer = ClientAgentSigner::default();
        let signer: Arc<dyn AgentSigner + Send + Sync> = Arc::new(signer);

        // Step 4: Connect to app interface
        tracing::debug!(
            "Connecting to app interface at {}",
            self.config.holochain_conductor_url
        );

        let app_ws = AppWebsocket::connect(
            &self.config.holochain_conductor_url,
            token_response.token,
            signer.clone(),
        )
        .await
        .map_err(|e| AppError::HolochainError(format!("App connection failed: {}", e)))?;

        // Step 5: Get app info to retrieve cell ID and agent pubkey
        let app_info: AppInfo = app_ws
            .app_info()
            .await
            .map_err(|e| AppError::HolochainError(format!("Failed to get app info: {}", e)))?
            .ok_or_else(|| AppError::HolochainError("App info not available".to_string()))?;

        // Find the main cell (assuming role name matches app pattern)
        let cell_id = self.find_cell_id(&app_info)?;
        let agent_pubkey = cell_id.agent_pubkey().clone();

        // Step 6: Authorize signing credentials for the agent
        // This allows the ClientAgentSigner to sign zome calls
        let _credentials = admin_ws
            .authorize_signing_credentials(holochain_client::AuthorizeSigningCredentialsPayload {
                cell_id: cell_id.clone(),
                functions: None, // All functions
            })
            .await
            .map_err(|e| AppError::HolochainError(format!("Failed to authorize signing: {}", e)))?;

        // Note: The credentials are automatically added to the signer during authorization
        tracing::debug!("Signing credentials authorized for agent");

        Ok((app_ws, agent_pubkey, cell_id))
    }

    /// Find the cell ID for the mycelix-mail DNA
    fn find_cell_id(&self, app_info: &AppInfo) -> AppResult<CellId> {
        // Look for cells in the app
        for (role_name, cells) in &app_info.cell_info {
            for cell_info in cells {
                match cell_info {
                    CellInfo::Provisioned(cell) => {
                        tracing::debug!("Found provisioned cell for role: {}", role_name);
                        return Ok(cell.cell_id.clone());
                    }
                    CellInfo::Cloned(cell) => {
                        tracing::debug!("Found cloned cell for role: {}", role_name);
                        return Ok(cell.cell_id.clone());
                    }
                    _ => continue,
                }
            }
        }

        Err(AppError::HolochainError(
            "No provisioned cell found in app".to_string(),
        ))
    }

    /// Attempt to reconnect to the conductor
    async fn try_reconnect(&self) -> AppResult<()> {
        let mut attempts = self.reconnect_attempts.write().await;

        if *attempts >= self.config.holochain_max_reconnect_attempts {
            return Err(AppError::HolochainError(format!(
                "Max reconnection attempts ({}) exceeded",
                self.config.holochain_max_reconnect_attempts
            )));
        }

        *attempts += 1;
        let current_attempt = *attempts;
        drop(attempts);

        tracing::info!(
            "Attempting reconnection ({}/{})",
            current_attempt,
            self.config.holochain_max_reconnect_attempts
        );

        // Mark as disconnected
        {
            let mut state = self.state.write().await;
            *state = ConnectionState::Disconnected;
        }

        // Exponential backoff
        let backoff = Duration::from_millis(100 * 2u64.pow(current_attempt - 1));
        tokio::time::sleep(backoff).await;

        self.connect().await
    }

    /// Check if connected
    pub async fn is_connected(&self) -> bool {
        let state = self.state.read().await;
        matches!(
            *state,
            ConnectionState::Connected { .. } | ConnectionState::StubMode { .. }
        )
    }

    /// Check if in stub mode
    pub async fn is_stub_mode(&self) -> bool {
        let state = self.state.read().await;
        matches!(*state, ConnectionState::StubMode { .. })
    }

    /// Get the agent's public key
    pub async fn get_agent_pubkey(&self) -> AppResult<AgentPubKey> {
        let state = self.state.read().await;
        match &*state {
            ConnectionState::Connected { agent_pubkey, .. } => Ok(agent_pubkey.clone()),
            ConnectionState::StubMode { mock_agent_pubkey } => Ok(mock_agent_pubkey.clone()),
            ConnectionState::Disconnected => {
                Err(AppError::HolochainError("Not connected".to_string()))
            }
        }
    }

    /// Generic zome call helper
    async fn call_zome<I, O>(
        &self,
        zome_name: &str,
        fn_name: &str,
        payload: I,
    ) -> AppResult<O>
    where
        I: Serialize + Debug + Clone,
        O: DeserializeOwned + Debug,
    {
        let state = self.state.read().await;

        match &*state {
            ConnectionState::Disconnected => {
                drop(state);
                // Try to reconnect
                self.try_reconnect().await?;
                // Retry the call
                return Box::pin(self.call_zome(zome_name, fn_name, payload)).await;
            }
            ConnectionState::StubMode { .. } => {
                tracing::warn!(
                    "STUB: call_zome({}, {}) - stub mode active",
                    zome_name,
                    fn_name
                );
                return Err(AppError::HolochainError(format!(
                    "Stub mode: zome call {}/{} not implemented",
                    zome_name, fn_name
                )));
            }
            ConnectionState::Connected { app_ws, cell_id, .. } => {
                tracing::debug!("Calling zome: {}/{}", zome_name, fn_name);

                // Clone payload in case we need to retry
                let payload_clone = payload.clone();

                // Encode the payload
                let encoded_payload = ExternIO::encode(payload)
                    .map_err(|e| AppError::HolochainError(format!("Failed to encode payload: {}", e)))?;

                // Make the zome call
                let result = app_ws
                    .call_zome(
                        cell_id.clone().into(),
                        zome_name.into(),
                        fn_name.into(),
                        encoded_payload,
                    )
                    .await;

                match result {
                    Ok(response) => {
                        // Decode the response
                        let decoded: O = response
                            .decode()
                            .map_err(|e| AppError::HolochainError(format!("Failed to decode response: {}", e)))?;

                        tracing::debug!("Zome call {}/{} succeeded", zome_name, fn_name);
                        Ok(decoded)
                    }
                    Err(e) => {
                        // Check if this is a connection error that warrants reconnection
                        let error_str = format!("{:?}", e);
                        if error_str.contains("connection") || error_str.contains("WebSocket") {
                            tracing::warn!("Connection error during zome call, will attempt reconnect");
                            drop(state);
                            self.try_reconnect().await?;
                            return Box::pin(self.call_zome::<I, O>(zome_name, fn_name, payload_clone)).await;
                        }

                        tracing::error!("Zome call {}/{} failed: {:?}", zome_name, fn_name, e);
                        Err(AppError::HolochainError(format!(
                            "Zome call failed: {:?}",
                            e
                        )))
                    }
                }
            }
        }
    }

    /// Stub helper that returns a default value for Vec types
    async fn call_zome_vec<I, O>(
        &self,
        zome_name: &str,
        fn_name: &str,
        payload: I,
    ) -> AppResult<Vec<O>>
    where
        I: Serialize + Debug + Clone,
        O: DeserializeOwned + Debug,
    {
        let state = self.state.read().await;

        if matches!(*state, ConnectionState::StubMode { .. }) {
            tracing::debug!(
                "STUB: call_zome_vec({}, {}) - returning empty vec",
                zome_name,
                fn_name
            );
            return Ok(Vec::new());
        }

        drop(state);
        self.call_zome(zome_name, fn_name, payload).await
    }

    /// Create a mock ActionHash for stub mode
    /// ActionHash prefix is [0x84, 0x29, 0x24] (uhCkk)
    fn mock_action_hash() -> ActionHash {
        let mut bytes = vec![0x84, 0x29, 0x24]; // ACTION_PREFIX
        bytes.extend(vec![0u8; 36]); // 36 zero bytes for the hash
        ActionHash::from_raw_39(bytes)
    }

    /// Create a mock AgentPubKey for stub mode
    /// AgentPubKey prefix is [0x84, 0x20, 0x24] (uhCAk)
    fn mock_agent_pubkey() -> AgentPubKey {
        let mut bytes = vec![0x84, 0x20, 0x24]; // AGENT_PREFIX
        bytes.extend(vec![0u8; 36]); // 36 zero bytes for the key
        AgentPubKey::from_raw_39(bytes)
    }

    // =========================================================================
    // DID Operations (via DHT - no external registry!)
    // =========================================================================

    /// Register a DID for the current agent
    pub async fn register_did(&self, did: &str) -> AppResult<ActionHash> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: register_did({}) - returning mock hash", did);
            return Ok(Self::mock_action_hash());
        }

        #[derive(Serialize, Debug, Clone)]
        struct RegisterDidInput {
            did: String,
        }

        self.call_zome(
            "mail_messages",
            "register_my_did",
            RegisterDidInput { did: did.to_string() },
        )
        .await
    }

    /// Resolve a DID to an AgentPubKey (via DHT)
    /// This replaces the Python DID Registry entirely
    pub async fn resolve_did(&self, did: &str) -> AppResult<AgentPubKey> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: resolve_did({}) - returning mock pubkey", did);
            return Ok(Self::mock_agent_pubkey());
        }

        #[derive(Serialize, Debug, Clone)]
        struct ResolveDidInput {
            did: String,
        }

        let binding: Option<DidBinding> = self
            .call_zome(
                "mail_messages",
                "resolve_did",
                ResolveDidInput { did: did.to_string() },
            )
            .await?;

        binding
            .map(|b| b.agent_pub_key)
            .ok_or_else(|| AppError::HolochainError("DID not found".to_string()))
    }

    // =========================================================================
    // Identity hApp Integration (mycelix-identity)
    // =========================================================================

    /// Check and setup identity hApp connection
    /// Called during connect() to discover if identity DNA is available
    #[allow(dead_code)]
    async fn setup_identity_connection(&self, app_info: &AppInfo) -> bool {
        // Look for identity role in the app info
        for (role_name, cells) in &app_info.cell_info {
            if role_name.contains("identity") {
                for cell_info in cells {
                    match cell_info {
                        CellInfo::Provisioned(cell) => {
                            let mut identity_cell = self.identity_cell_id.write().await;
                            *identity_cell = Some(cell.cell_id.clone());
                            let mut available = self.identity_happ_available.write().await;
                            *available = true;
                            tracing::info!(
                                "Identity hApp connected via role: {}",
                                role_name
                            );
                            return true;
                        }
                        CellInfo::Cloned(cell) => {
                            let mut identity_cell = self.identity_cell_id.write().await;
                            *identity_cell = Some(cell.cell_id.clone());
                            let mut available = self.identity_happ_available.write().await;
                            *available = true;
                            tracing::info!(
                                "Identity hApp connected (cloned) via role: {}",
                                role_name
                            );
                            return true;
                        }
                        _ => continue,
                    }
                }
            }
        }

        tracing::warn!(
            "Identity hApp not found - identity features will be degraded"
        );
        false
    }

    /// Check if identity hApp is available
    pub async fn is_identity_available(&self) -> bool {
        *self.identity_happ_available.read().await
    }

    /// Make a zome call to the identity DNA
    async fn call_identity_zome<I, O>(
        &self,
        zome_name: &str,
        fn_name: &str,
        payload: I,
    ) -> AppResult<O>
    where
        I: Serialize + Debug + Clone,
        O: DeserializeOwned + Debug,
    {
        let state = self.state.read().await;
        let identity_cell = self.identity_cell_id.read().await;

        match (&*state, &*identity_cell) {
            (ConnectionState::Connected { app_ws, .. }, Some(cell_id)) => {
                tracing::debug!("Calling identity zome: {}/{}", zome_name, fn_name);

                let encoded_payload = ExternIO::encode(payload)
                    .map_err(|e| AppError::HolochainError(format!("Failed to encode payload: {}", e)))?;

                let result = app_ws
                    .call_zome(
                        cell_id.clone().into(),
                        zome_name.into(),
                        fn_name.into(),
                        encoded_payload,
                    )
                    .await;

                match result {
                    Ok(response) => {
                        let decoded: O = response
                            .decode()
                            .map_err(|e| AppError::HolochainError(format!("Failed to decode response: {}", e)))?;
                        Ok(decoded)
                    }
                    Err(e) => {
                        tracing::error!("Identity zome call {}/{} failed: {:?}", zome_name, fn_name, e);
                        Err(AppError::HolochainError(format!("Identity zome call failed: {:?}", e)))
                    }
                }
            }
            (ConnectionState::StubMode { .. }, _) => {
                Err(AppError::HolochainError(format!(
                    "Stub mode: identity zome call {}/{} not available",
                    zome_name, fn_name
                )))
            }
            _ => {
                Err(AppError::HolochainError("Identity hApp not available".to_string()))
            }
        }
    }

    /// Resolve a DID from the identity hApp
    /// Returns the full DID document with verification methods
    pub async fn resolve_did_from_identity_happ(&self, did: &str) -> AppResult<DidResolutionResult> {
        // Check cache first (with TTL enforcement and expired entry cleanup)
        {
            let now = Instant::now();
            let mut cache = self.did_cache.write().await;

            if let Some(entry) = cache.get(did) {
                if entry.expires_at > now {
                    tracing::debug!("DID cache hit: {}", did);
                    return Ok(DidResolutionResult {
                        success: true,
                        did_document: Some(entry.document.clone()),
                        error: None,
                        cached: true,
                    });
                } else {
                    // Expired — remove stale entry so revoked DIDs don't linger
                    cache.remove(did);
                    tracing::debug!("DID cache expired, removed: {}", did);
                }
            }

            // Periodic cleanup: evict all expired entries if cache is large
            if cache.len() > 1000 {
                cache.retain(|_, entry| entry.expires_at > now);
            }
        }

        // Stub mode fallback
        if self.is_stub_mode().await {
            tracing::debug!("STUB: resolve_did_from_identity_happ({}) - returning mock", did);
            return Ok(DidResolutionResult {
                success: true,
                did_document: Some(DidDocument {
                    id: did.to_string(),
                    controller: "stub-controller".to_string(),
                    verification_method: vec![],
                    authentication: vec![],
                    service: vec![],
                    created: 0,
                    updated: 0,
                    version: 1,
                }),
                error: None,
                cached: false,
            });
        }

        // Check if identity hApp is available
        if !self.is_identity_available().await {
            tracing::warn!(
                "Identity hApp not available - returning degraded DID resolution for {}",
                did
            );
            return Ok(DidResolutionResult {
                success: false,
                did_document: None,
                error: Some("Identity hApp not available - using degraded mode".to_string()),
                cached: false,
            });
        }

        // Make the real zome call
        match self.call_identity_zome::<_, DidResolutionRecord>("did_registry", "resolve_did", did.to_string()).await {
            Ok(record) => {
                let doc = record.into_did_document();

                // Cache the result
                {
                    let mut cache = self.did_cache.write().await;
                    cache.insert(did.to_string(), DidCacheEntry {
                        document: doc.clone(),
                        expires_at: Instant::now() + self.cache_ttl,
                    });
                }

                Ok(DidResolutionResult {
                    success: true,
                    did_document: Some(doc),
                    error: None,
                    cached: false,
                })
            }
            Err(e) => {
                tracing::warn!("DID resolution failed for {}: {}", did, e);
                Ok(DidResolutionResult {
                    success: false,
                    did_document: None,
                    error: Some(e.to_string()),
                    cached: false,
                })
            }
        }
    }

    /// Verify a credential using the identity hApp
    pub async fn verify_credential(&self, credential_id: &str) -> AppResult<CredentialVerificationResult> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: verify_credential({}) - returning valid", credential_id);
            return Ok(CredentialVerificationResult {
                valid: true,
                checks: VerificationChecks {
                    signature_valid: true,
                    not_expired: true,
                    not_revoked: true,
                    issuer_trusted: true,
                    schema_valid: true,
                },
                error: None,
                assurance_level: Some(AssuranceLevel::E1),
            });
        }

        if !self.is_identity_available().await {
            tracing::warn!(
                "Identity hApp not available - returning permissive credential verification for {}",
                credential_id
            );
            return Ok(CredentialVerificationResult {
                valid: true, // Permissive in degraded mode
                checks: VerificationChecks::default_permissive(),
                error: Some("Identity hApp not available - verification skipped".to_string()),
                assurance_level: Some(AssuranceLevel::E0),
            });
        }

        // Check revocation status
        let revocation = self.check_credential_revocation(credential_id).await?;

        Ok(CredentialVerificationResult {
            valid: revocation.status == RevocationState::Active,
            checks: VerificationChecks {
                signature_valid: true, // Would verify in production
                not_expired: true,     // Would check expiration
                not_revoked: revocation.status == RevocationState::Active,
                issuer_trusted: true,  // Would verify issuer
                schema_valid: true,    // Would validate schema
            },
            error: if revocation.status != RevocationState::Active {
                Some(format!("Credential is {:?}", revocation.status))
            } else {
                None
            },
            assurance_level: Some(AssuranceLevel::E1),
        })
    }

    /// Check credential revocation status
    pub async fn check_credential_revocation(&self, credential_id: &str) -> AppResult<RevocationStatusResult> {
        if self.is_stub_mode().await {
            return Ok(RevocationStatusResult {
                credential_id: credential_id.to_string(),
                status: RevocationState::Active,
                reason: None,
                checked_at: current_timestamp(),
            });
        }

        if !self.is_identity_available().await {
            // Permissive default - assume active
            return Ok(RevocationStatusResult {
                credential_id: credential_id.to_string(),
                status: RevocationState::Active,
                reason: None,
                checked_at: current_timestamp(),
            });
        }

        match self.call_identity_zome::<_, RevocationCheckOutput>(
            "revocation",
            "check_revocation_status",
            credential_id.to_string()
        ).await {
            Ok(result) => Ok(RevocationStatusResult {
                credential_id: result.credential_id,
                status: result.status.into(),
                reason: result.reason,
                checked_at: current_timestamp(),
            }),
            Err(e) => {
                tracing::warn!("Revocation check failed for {}: {}", credential_id, e);
                // Permissive fallback
                Ok(RevocationStatusResult {
                    credential_id: credential_id.to_string(),
                    status: RevocationState::Active,
                    reason: None,
                    checked_at: current_timestamp(),
                })
            }
        }
    }

    /// Get the assurance level for a DID
    /// Returns E0-E4 based on verified credentials
    pub async fn get_assurance_level(&self, did: &str) -> AppResult<AssuranceLevel> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: get_assurance_level({}) - returning E1", did);
            return Ok(AssuranceLevel::E1);
        }

        if !self.is_identity_available().await {
            tracing::warn!(
                "Identity hApp not available - returning E0 for {}",
                did
            );
            return Ok(AssuranceLevel::E0);
        }

        // Try to get assurance level from identity bridge
        match self.call_identity_zome::<_, AssuranceLevelOutput>(
            "identity_bridge",
            "get_assurance_level",
            did.to_string()
        ).await {
            Ok(result) => Ok(result.level.into()),
            Err(e) => {
                tracing::warn!("Assurance level check failed for {}: {}", did, e);
                Ok(AssuranceLevel::E0)
            }
        }
    }

    /// Check if a DID meets a minimum assurance level
    pub async fn meets_assurance_level(&self, did: &str, min_level: AssuranceLevel) -> AppResult<bool> {
        let current = self.get_assurance_level(did).await?;
        Ok(current.value() >= min_level.value())
    }

    /// Verify identity before sending a message
    /// Returns Ok if the sender is allowed to send, Err with reason if not
    pub async fn verify_sender_for_message(&self, sender_did: &str) -> AppResult<SenderVerificationResult> {
        // Get assurance level
        let assurance = self.get_assurance_level(sender_did).await?;

        // Resolve DID to check if valid
        let resolution = self.resolve_did_from_identity_happ(sender_did).await?;

        // Check trust score via MATL
        let trust = self.evaluate_trust_matl(sender_did).await?;

        // Determine if allowed to send
        let allowed = resolution.success && !trust.is_byzantine;

        let warnings: Vec<String> = {
            let mut w = vec![];
            if assurance == AssuranceLevel::E0 {
                w.push("Sender has unverified identity (E0)".to_string());
            }
            if trust.is_new_user {
                w.push("Sender is a new user with limited history".to_string());
            }
            if trust.score < 0.3 {
                w.push(format!("Sender has low trust score: {:.2}", trust.score));
            }
            w
        };

        Ok(SenderVerificationResult {
            allowed,
            assurance_level: assurance,
            trust_score: trust.score,
            is_byzantine: trust.is_byzantine,
            warnings,
        })
    }

    /// Clear the DID cache
    pub async fn clear_did_cache(&self) {
        let mut cache = self.did_cache.write().await;
        cache.clear();
        tracing::debug!("DID cache cleared");
    }

    // =========================================================================
    // Mail Operations
    // =========================================================================

    /// Send an email message
    pub async fn send_message(&self, message: MailMessageInput) -> AppResult<ActionHash> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: send_message to {} - returning mock hash", message.to_did);
            return Ok(Self::mock_action_hash());
        }

        self.call_zome("mail_messages", "send_message", message).await
    }

    /// Get inbox messages
    pub async fn get_inbox(&self) -> AppResult<Vec<MailMessageOutput>> {
        self.call_zome_vec("mail_messages", "get_inbox", ()).await
    }

    /// Get outbox (sent) messages
    pub async fn get_outbox(&self) -> AppResult<Vec<MailMessageOutput>> {
        self.call_zome_vec("mail_messages", "get_outbox", ()).await
    }

    /// Get a single message by hash
    pub async fn get_message(&self, hash: ActionHash) -> AppResult<Option<MailMessageOutput>> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: get_message - returning None");
            return Ok(None);
        }

        #[derive(Serialize, Debug, Clone)]
        struct GetMessageInput {
            hash: ActionHash,
        }

        self.call_zome("mail_messages", "get_message", GetMessageInput { hash })
            .await
    }

    /// Get thread messages
    pub async fn get_thread(&self, parent_hash: ActionHash) -> AppResult<Vec<MailMessageOutput>> {
        if self.is_stub_mode().await {
            return Ok(Vec::new());
        }

        #[derive(Serialize, Debug, Clone)]
        struct GetThreadInput {
            parent_hash: ActionHash,
        }

        self.call_zome("mail_messages", "get_thread", GetThreadInput { parent_hash })
            .await
    }

    /// Delete a message
    pub async fn delete_message(&self, hash: ActionHash) -> AppResult<ActionHash> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: delete_message - returning mock hash");
            return Ok(Self::mock_action_hash());
        }

        #[derive(Serialize, Debug, Clone)]
        struct DeleteMessageInput {
            hash: ActionHash,
        }

        self.call_zome("mail_messages", "delete_message", DeleteMessageInput { hash })
            .await
    }

    // =========================================================================
    // Trust Operations
    // =========================================================================

    /// Check trust score for a DID
    pub async fn check_sender_trust(&self, did: &str) -> AppResult<f64> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: check_sender_trust({}) - returning 0.5", did);
            return Ok(0.5);
        }

        #[derive(Serialize, Debug, Clone)]
        struct CheckTrustInput {
            did: String,
        }

        self.call_zome(
            "trust_filter",
            "check_sender_trust",
            CheckTrustInput { did: did.to_string() },
        )
        .await
    }

    /// Get filtered inbox (by trust score)
    pub async fn filter_inbox(&self, min_trust: f64) -> AppResult<Vec<MailMessageOutput>> {
        if self.is_stub_mode().await {
            return Ok(Vec::new());
        }

        #[derive(Serialize, Debug, Clone)]
        struct FilterInboxInput {
            min_trust: f64,
        }

        self.call_zome("trust_filter", "filter_inbox", FilterInboxInput { min_trust })
            .await
    }

    /// Get filtered inbox with Byzantine detection
    pub async fn filter_inbox_matl(&self, min_trust: f64) -> AppResult<FilteredInboxResult> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: filter_inbox_matl({}) - returning empty", min_trust);
            return Ok(FilteredInboxResult::default());
        }

        #[derive(Serialize, Debug, Clone)]
        struct FilterInboxMatlInput {
            min_trust: f64,
        }

        self.call_zome("trust_filter", "filter_inbox_matl", FilterInboxMatlInput { min_trust })
            .await
    }

    /// Update trust score
    pub async fn update_trust_score(&self, trust: TrustScoreInput) -> AppResult<ActionHash> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: update_trust_score({}) - returning mock hash", trust.did);
            return Ok(Self::mock_action_hash());
        }

        self.call_zome("trust_filter", "update_trust_score", trust).await
    }

    /// Report spam
    pub async fn report_spam(&self, input: SpamReportInput) -> AppResult<()> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: report_spam({}) - no-op", input.reason);
            return Ok(());
        }

        self.call_zome("trust_filter", "report_spam", input).await
    }

    /// Record positive interaction (increases trust)
    pub async fn record_positive_interaction(&self, did: &str) -> AppResult<ActionHash> {
        if self.is_stub_mode().await {
            tracing::debug!(
                "STUB: record_positive_interaction({}) - returning mock hash",
                did
            );
            return Ok(Self::mock_action_hash());
        }

        #[derive(Serialize, Debug, Clone)]
        struct PositiveInteractionInput {
            did: String,
        }

        self.call_zome(
            "trust_filter",
            "record_positive_interaction",
            PositiveInteractionInput { did: did.to_string() },
        )
        .await
    }

    /// Get cross-hApp reputation
    pub async fn get_cross_happ_reputation(&self, did: &str) -> AppResult<CrossHappReputation> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: get_cross_happ_reputation({}) - returning default", did);
            return Ok(CrossHappReputation {
                did: did.to_string(),
                local_score: 0.5,
                cross_happ_scores: vec![],
                aggregate: 0.5,
            });
        }

        #[derive(Serialize, Debug, Clone)]
        struct GetReputationInput {
            did: String,
        }

        self.call_zome(
            "trust_filter",
            "get_cross_happ_reputation",
            GetReputationInput { did: did.to_string() },
        )
        .await
    }

    /// Evaluate trust using MATL
    pub async fn evaluate_trust_matl(&self, did: &str) -> AppResult<MatlTrustResult> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: evaluate_trust_matl({}) - returning default", did);
            return Ok(MatlTrustResult {
                did: did.to_string(),
                score: 0.5,
                is_byzantine: false,
                interaction_count: 0,
                is_new_user: true,
            });
        }

        #[derive(Serialize, Debug, Clone)]
        struct EvaluateTrustInput {
            did: String,
        }

        self.call_zome(
            "trust_filter",
            "evaluate_trust_matl",
            EvaluateTrustInput { did: did.to_string() },
        )
        .await
    }

    // =========================================================================
    // Contact Operations
    // =========================================================================

    /// Add a contact
    pub async fn add_contact(&self, contact: ContactInput) -> AppResult<ActionHash> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: add_contact({}) - returning mock hash", contact.did);
            return Ok(Self::mock_action_hash());
        }

        self.call_zome("mail_messages", "add_contact", contact).await
    }

    /// Get all contacts
    pub async fn get_contacts(&self) -> AppResult<Vec<ContactOutput>> {
        self.call_zome_vec("mail_messages", "get_contacts", ()).await
    }

    // =========================================================================
    // Bridge Zome Operations (Cross-hApp Reputation)
    // =========================================================================

    /// Query cross-hApp reputation from Bridge zome
    ///
    /// Aggregates reputation scores from multiple hApps for ecosystem-wide trust.
    pub async fn bridge_aggregate_reputation(&self, agent: &str) -> AppResult<BridgeAggregateReputation> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: bridge_aggregate_reputation({}) - returning default", agent);
            return Ok(BridgeAggregateReputation {
                agent: agent.to_string(),
                aggregate: 0.5,
                scores: vec![],
                total_interactions: 0,
                is_trustworthy: true,
            });
        }

        self.call_zome("bridge", "aggregate_cross_happ_reputation", agent.to_string())
            .await
    }

    /// Query reputation records for an agent from Bridge zome
    pub async fn bridge_query_reputation(&self, agent: &str, happ_filter: Option<&str>) -> AppResult<Vec<BridgeReputationRecord>> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: bridge_query_reputation({}) - returning empty", agent);
            return Ok(Vec::new());
        }

        #[derive(Serialize, Debug, Clone)]
        struct QueryInput {
            agent: String,
            happ: Option<String>,
        }

        self.call_zome("bridge", "query_reputation", QueryInput {
            agent: agent.to_string(),
            happ: happ_filter.map(|s| s.to_string()),
        })
        .await
    }

    /// Record reputation for an agent via Bridge zome
    pub async fn bridge_record_reputation(&self, input: BridgeRecordReputationInput) -> AppResult<ActionHash> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: bridge_record_reputation({}) - returning mock hash", input.agent);
            return Ok(Self::mock_action_hash());
        }

        self.call_zome("bridge", "record_reputation", input).await
    }

    /// Check if an agent is trustworthy via Bridge zome
    pub async fn bridge_is_trustworthy(&self, agent: &str, threshold: f64) -> AppResult<bool> {
        if self.is_stub_mode().await {
            tracing::debug!("STUB: bridge_is_trustworthy({}, {}) - returning true", agent, threshold);
            return Ok(true);
        }

        #[derive(Serialize, Debug, Clone)]
        struct TrustCheckInput {
            agent: String,
            threshold: f64,
        }

        self.call_zome("bridge", "is_agent_trustworthy", TrustCheckInput {
            agent: agent.to_string(),
            threshold,
        })
        .await
    }

    // =========================================================================
    // Connection Management
    // =========================================================================

    /// Disconnect from the conductor
    pub async fn disconnect(&self) {
        let mut state = self.state.write().await;
        *state = ConnectionState::Disconnected;
        tracing::info!("Disconnected from Holochain conductor");
    }

    /// Get connection status information
    pub async fn connection_info(&self) -> ConnectionInfo {
        let state = self.state.read().await;
        let reconnect_attempts = *self.reconnect_attempts.read().await;

        match &*state {
            ConnectionState::Disconnected => ConnectionInfo {
                status: "disconnected".to_string(),
                mode: None,
                agent_pubkey: None,
                reconnect_attempts,
            },
            ConnectionState::StubMode { mock_agent_pubkey } => ConnectionInfo {
                status: "connected".to_string(),
                mode: Some("stub".to_string()),
                agent_pubkey: Some(format!("{:?}", mock_agent_pubkey)),
                reconnect_attempts,
            },
            ConnectionState::Connected { agent_pubkey, .. } => ConnectionInfo {
                status: "connected".to_string(),
                mode: Some("production".to_string()),
                agent_pubkey: Some(format!("{:?}", agent_pubkey)),
                reconnect_attempts,
            },
        }
    }
}

/// Connection status information
#[derive(Debug, Clone, serde::Serialize)]
pub struct ConnectionInfo {
    pub status: String,
    pub mode: Option<String>,
    pub agent_pubkey: Option<String>,
    pub reconnect_attempts: u32,
}

// =========================================================================
// Zome Input/Output Types (match integrity zome)
// =========================================================================

#[derive(Debug, Clone, Serialize)]
pub struct MailMessageInput {
    pub from_did: String,
    pub to_did: String,
    pub subject_encrypted: Vec<u8>,
    pub body_cid: String,
    pub timestamp: i64,
    pub thread_id: Option<String>,
    pub epistemic_tier: String,
}

#[derive(Debug, Default, serde::Deserialize)]
pub struct MailMessageOutput {
    #[serde(default)]
    pub from_did: String,
    #[serde(default)]
    pub to_did: String,
    #[serde(default)]
    pub subject_encrypted: Vec<u8>,
    #[serde(default)]
    pub body_cid: String,
    #[serde(default)]
    pub timestamp: i64,
    #[serde(default)]
    pub thread_id: Option<String>,
    #[serde(default)]
    pub epistemic_tier: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct TrustScoreInput {
    pub did: String,
    pub score: f64,
    pub source: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct SpamReportInput {
    pub message_hash: ActionHash,
    pub reason: String,
}

#[derive(Debug, Default, serde::Deserialize)]
pub struct FilteredInboxResult {
    #[serde(default)]
    pub messages: Vec<MailMessageOutput>,
    #[serde(default)]
    pub byzantine_senders: Vec<String>,
}

#[derive(Debug, Default, serde::Deserialize)]
pub struct CrossHappReputation {
    #[serde(default)]
    pub did: String,
    #[serde(default)]
    pub local_score: f64,
    #[serde(default)]
    pub cross_happ_scores: Vec<HappScore>,
    #[serde(default)]
    pub aggregate: f64,
}

#[derive(Debug, Default, serde::Deserialize)]
pub struct HappScore {
    #[serde(default)]
    pub happ_id: String,
    #[serde(default)]
    pub score: f64,
}

#[derive(Debug, Default, serde::Deserialize)]
pub struct MatlTrustResult {
    #[serde(default)]
    pub did: String,
    #[serde(default)]
    pub score: f64,
    #[serde(default)]
    pub is_byzantine: bool,
    #[serde(default)]
    pub interaction_count: u64,
    #[serde(default)]
    pub is_new_user: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct ContactInput {
    pub name: String,
    pub did: String,
    pub email_alias: Option<String>,
    pub notes: Option<String>,
}

#[derive(Debug, Default, serde::Deserialize)]
pub struct ContactOutput {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub did: String,
    #[serde(default)]
    pub email_alias: Option<String>,
    #[serde(default)]
    pub notes: Option<String>,
    #[serde(default)]
    pub added_at: i64,
}

// =========================================================================
// Bridge Zome Types (cross-hApp reputation)
// =========================================================================

/// Input for recording reputation via Bridge zome
#[derive(Debug, Clone, Serialize)]
pub struct BridgeRecordReputationInput {
    pub agent: String,
    pub happ_id: String,
    pub happ_name: String,
    pub score: f64,
    pub interactions: u64,
    pub negative_interactions: u64,
    pub evidence_hash: Option<String>,
}

/// Output from Bridge zome reputation query
#[derive(Debug, Clone, Default, serde::Deserialize)]
pub struct BridgeReputationRecord {
    #[serde(default)]
    pub agent: String,
    #[serde(default)]
    pub happ_id: String,
    #[serde(default)]
    pub happ_name: String,
    #[serde(default)]
    pub score: f64,
    #[serde(default)]
    pub interactions: u64,
    #[serde(default)]
    pub negative_interactions: u64,
    #[serde(default)]
    pub updated_at: u64,
}

/// Aggregate reputation from Bridge zome
#[derive(Debug, Clone, Default, serde::Deserialize)]
pub struct BridgeAggregateReputation {
    #[serde(default)]
    pub agent: String,
    #[serde(default)]
    pub aggregate: f64,
    #[serde(default)]
    pub scores: Vec<BridgeHappScore>,
    #[serde(default)]
    pub total_interactions: u64,
    #[serde(default)]
    pub is_trustworthy: bool,
}

/// Individual hApp score from Bridge
#[derive(Debug, Clone, Default, serde::Deserialize)]
pub struct BridgeHappScore {
    #[serde(default)]
    pub happ_id: String,
    #[serde(default)]
    pub happ_name: String,
    #[serde(default)]
    pub score: f64,
    #[serde(default)]
    pub interactions: u64,
    #[serde(default)]
    pub last_updated: u64,
}

// =========================================================================
// Identity Types (mycelix-identity integration)
// =========================================================================

/// Assurance levels following eIDAS-inspired tiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, serde::Deserialize, Default)]
pub enum AssuranceLevel {
    /// Unverified (self-attested only)
    #[default]
    E0,
    /// Email verified
    E1,
    /// Phone verified + social recovery configured
    E2,
    /// Government ID verified (KYC)
    E3,
    /// Biometric + multi-factor (high-value transactions)
    E4,
}

impl AssuranceLevel {
    /// Get numeric value for comparisons
    pub fn value(&self) -> u8 {
        match self {
            AssuranceLevel::E0 => 0,
            AssuranceLevel::E1 => 1,
            AssuranceLevel::E2 => 2,
            AssuranceLevel::E3 => 3,
            AssuranceLevel::E4 => 4,
        }
    }

    /// Get human-readable description
    pub fn description(&self) -> &'static str {
        match self {
            AssuranceLevel::E0 => "Unverified",
            AssuranceLevel::E1 => "Email Verified",
            AssuranceLevel::E2 => "Phone + Recovery",
            AssuranceLevel::E3 => "Government ID",
            AssuranceLevel::E4 => "Biometric + MFA",
        }
    }
}

impl From<String> for AssuranceLevel {
    fn from(s: String) -> Self {
        match s.to_uppercase().as_str() {
            "E1" => AssuranceLevel::E1,
            "E2" => AssuranceLevel::E2,
            "E3" => AssuranceLevel::E3,
            "E4" => AssuranceLevel::E4,
            _ => AssuranceLevel::E0,
        }
    }
}

/// DID Document structure (W3C DID Core compatible)
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct DidDocument {
    /// DID identifier (did:mycelix:...)
    pub id: String,
    /// Controller agent pub key
    pub controller: String,
    /// Verification methods (keys)
    pub verification_method: Vec<VerificationMethod>,
    /// Authentication methods
    pub authentication: Vec<String>,
    /// Service endpoints
    pub service: Vec<ServiceEndpoint>,
    /// Creation timestamp
    pub created: i64,
    /// Last update timestamp
    pub updated: i64,
    /// Document version
    pub version: u32,
}

/// Verification method (key) in a DID document
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct VerificationMethod {
    pub id: String,
    #[serde(rename = "type")]
    pub type_: String,
    pub controller: String,
    pub public_key_multibase: String,
}

/// Service endpoint in a DID document
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct ServiceEndpoint {
    pub id: String,
    #[serde(rename = "type")]
    pub type_: String,
    pub service_endpoint: String,
}

/// Result of DID resolution
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct DidResolutionResult {
    pub success: bool,
    pub did_document: Option<DidDocument>,
    pub error: Option<String>,
    pub cached: bool,
}

/// Internal type for deserializing zome response
#[derive(Debug, serde::Deserialize)]
struct DidResolutionRecord {
    #[serde(default)]
    id: String,
    #[serde(default)]
    controller: String,
    #[serde(default)]
    verification_method: Vec<VerificationMethodRaw>,
    #[serde(default)]
    authentication: Vec<String>,
    #[serde(default)]
    service: Vec<ServiceEndpointRaw>,
    #[serde(default)]
    created: i64,
    #[serde(default)]
    updated: i64,
    #[serde(default)]
    version: u32,
}

#[derive(Debug, serde::Deserialize)]
struct VerificationMethodRaw {
    id: String,
    #[serde(rename = "type")]
    type_: Option<String>,
    controller: String,
    public_key_multibase: Option<String>,
}

#[derive(Debug, serde::Deserialize)]
struct ServiceEndpointRaw {
    id: String,
    #[serde(rename = "type")]
    type_: Option<String>,
    service_endpoint: String,
}

impl DidResolutionRecord {
    fn into_did_document(self) -> DidDocument {
        DidDocument {
            id: self.id,
            controller: self.controller,
            verification_method: self.verification_method.into_iter().map(|vm| VerificationMethod {
                id: vm.id,
                type_: vm.type_.unwrap_or_else(|| "Ed25519VerificationKey2020".to_string()),
                controller: vm.controller,
                public_key_multibase: vm.public_key_multibase.unwrap_or_default(),
            }).collect(),
            authentication: self.authentication,
            service: self.service.into_iter().map(|s| ServiceEndpoint {
                id: s.id,
                type_: s.type_.unwrap_or_else(|| "LinkedDomains".to_string()),
                service_endpoint: s.service_endpoint,
            }).collect(),
            created: self.created,
            updated: self.updated,
            version: self.version,
        }
    }
}

/// Credential verification result
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct CredentialVerificationResult {
    pub valid: bool,
    pub checks: VerificationChecks,
    pub error: Option<String>,
    pub assurance_level: Option<AssuranceLevel>,
}

/// Verification checks performed
#[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
pub struct VerificationChecks {
    pub signature_valid: bool,
    pub not_expired: bool,
    pub not_revoked: bool,
    pub issuer_trusted: bool,
    pub schema_valid: bool,
}

impl VerificationChecks {
    /// Default permissive checks (for degraded mode)
    fn default_permissive() -> Self {
        Self {
            signature_valid: true,
            not_expired: true,
            not_revoked: true,
            issuer_trusted: true,
            schema_valid: true,
        }
    }
}

/// Revocation status result
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct RevocationStatusResult {
    pub credential_id: String,
    pub status: RevocationState,
    pub reason: Option<String>,
    pub checked_at: i64,
}

/// Revocation state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, serde::Deserialize, Default)]
pub enum RevocationState {
    #[default]
    Active,
    Revoked,
    Suspended,
}

impl From<String> for RevocationState {
    fn from(s: String) -> Self {
        match s.to_lowercase().as_str() {
            "revoked" => RevocationState::Revoked,
            "suspended" => RevocationState::Suspended,
            _ => RevocationState::Active,
        }
    }
}

/// Internal type for revocation check output
#[derive(Debug, serde::Deserialize)]
struct RevocationCheckOutput {
    #[serde(default)]
    credential_id: String,
    #[serde(default)]
    status: String,
    #[serde(default)]
    reason: Option<String>,
}

/// Internal type for assurance level output
#[derive(Debug, serde::Deserialize)]
struct AssuranceLevelOutput {
    #[serde(default)]
    level: String,
}

/// Result of sender verification before message
#[derive(Debug, Clone, Serialize, serde::Deserialize)]
pub struct SenderVerificationResult {
    pub allowed: bool,
    pub assurance_level: AssuranceLevel,
    pub trust_score: f64,
    pub is_byzantine: bool,
    pub warnings: Vec<String>,
}

/// Helper function to get current timestamp
fn current_timestamp() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a test config with stub mode enabled
    fn test_config() -> Config {
        Config {
            host: "0.0.0.0".to_string(),
            port: 3001,
            holochain_conductor_url: "ws://localhost:8888".to_string(),
            holochain_admin_url: "ws://localhost:8889".to_string(),
            holochain_app_id: "mycelix_mail".to_string(),
            holochain_stub_mode: true,
            lair_url: None,
            lair_passphrase: None,
            holochain_connect_timeout_secs: 30,
            holochain_max_reconnect_attempts: 5,
            jwt_secret: "test_secret_for_testing_only_32chars!".to_string(),
            jwt_expiration_hours: 24,
            trust_cache_ttl_secs: 300,
            trust_cache_max_entries: 10000,
            default_min_trust: 0.3,
            byzantine_threshold: 0.2,
            cors_origins: vec!["http://localhost:3000".to_string()],
            rate_limit_rpm: 100,
            log_level: "info".to_string(),
            // Bridge configuration
            bridge_url: None,
            bridge_stub_mode: true,
            bridge_cache_ttl_secs: 300,
            bridge_fallback_trust: 0.3,
            bridge_zome_name: "bridge".to_string(),
            bridge_cross_happ_enabled: true,
            bridge_min_confidence: 0.3,
            identity_conductor_url: None,
            identity_verify_on_send: true,
            mail_kem_secret_key: None,
        }
    }

    #[tokio::test]
    async fn test_stub_mode_initialization() {
        let config = test_config();
        let service = HolochainService::new(config);

        // Connect should succeed in stub mode
        service.connect().await.expect("Connect should succeed in stub mode");

        assert!(service.is_connected().await);
        assert!(service.is_stub_mode().await);
    }

    #[tokio::test]
    async fn test_stub_mode_operations() {
        let config = test_config();
        let service = HolochainService::new(config);
        service.connect().await.expect("Connect should succeed");

        // Test that stub operations return expected values
        let trust = service.check_sender_trust("did:test:123").await.expect("Should return stub trust");
        assert_eq!(trust, 0.5);

        let inbox = service.get_inbox().await.expect("Should return empty inbox");
        assert!(inbox.is_empty());
    }

    #[tokio::test]
    async fn test_connection_info() {
        let config = test_config();
        let service = HolochainService::new(config);

        // Before connection
        let info = service.connection_info().await;
        assert_eq!(info.status, "disconnected");

        // After connection
        service.connect().await.expect("Connect should succeed");
        let info = service.connection_info().await;
        assert_eq!(info.status, "connected");
        assert_eq!(info.mode, Some("stub".to_string()));
    }

    #[tokio::test]
    async fn test_stub_did_operations() {
        let config = test_config();
        let service = HolochainService::new(config);
        service.connect().await.expect("Connect should succeed");

        // Register DID should return mock hash
        let hash = service.register_did("did:test:abc").await.expect("Should return mock hash");
        assert_eq!(hash.get_raw_39().len(), 39);

        // Resolve DID should return mock pubkey
        let pubkey = service.resolve_did("did:test:abc").await.expect("Should return mock pubkey");
        assert_eq!(pubkey.get_raw_39().len(), 39);
    }

    #[tokio::test]
    async fn test_stub_trust_operations() {
        let config = test_config();
        let service = HolochainService::new(config);
        service.connect().await.expect("Connect should succeed");

        // MATL trust evaluation
        let result = service.evaluate_trust_matl("did:test:alice").await.expect("Should return MATL result");
        assert_eq!(result.score, 0.5);
        assert!(!result.is_byzantine);
        assert!(result.is_new_user);

        // Cross-hApp reputation
        let rep = service.get_cross_happ_reputation("did:test:bob").await.expect("Should return reputation");
        assert_eq!(rep.aggregate, 0.5);
    }
}
