// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Bridge Client Service
//!
//! Provides cross-hApp communication for identity queries and reputation
//! in the Mycelix ecosystem. Integrates with the Bridge zome to enable
//! federated trust scoring across applications.
//!
//! The Bridge enables Mail to:
//! - Query identity information from the Identity hApp
//! - Get cross-hApp reputation scores for sender verification
//! - Report positive/negative interactions to the reputation system
//! - Participate in the ecosystem-wide trust network
//!
//! ## Bridge Zome Integration
//!
//! When connected to a real Holochain conductor with the Bridge zome installed,
//! this service makes actual zome calls to:
//! - `bridge` coordinator zome: `aggregate_cross_happ_reputation`, `record_reputation`, `query_reputation`
//! - `identity_bridge` coordinator zome: `resolve_did`, `get_identity_info`
//!
//! In stub mode or when Bridge is unavailable, it falls back to sensible defaults.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::RwLock;
use serde::{Deserialize, Serialize};

use crate::config::Config;
use crate::error::{AppError, AppResult};

// ============================================================================
// Bridge Zome Input/Output Types
// ============================================================================

/// Input for querying reputation from Bridge zome
#[derive(Debug, Clone, Serialize)]
pub struct QueryReputationInput {
    pub agent: String,
    pub happ: Option<String>,
}

/// Input for recording reputation via Bridge zome
#[derive(Debug, Clone, Serialize)]
pub struct RecordReputationInput {
    pub agent: String,
    pub happ_id: String,
    pub happ_name: String,
    pub score: f64,
    pub interactions: u64,
    pub negative_interactions: u64,
    pub evidence_hash: Option<String>,
}

/// Output from Bridge zome reputation query
#[derive(Debug, Clone, Deserialize, Default)]
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

/// Output from Bridge zome aggregate reputation query
#[derive(Debug, Clone, Deserialize, Default)]
pub struct BridgeCrossHappReputation {
    #[serde(default)]
    pub agent: String,
    #[serde(default)]
    pub aggregate: f64,
    #[serde(default)]
    pub scores: Vec<BridgeHappScore>,
    #[serde(default)]
    pub total_interactions: u64,
}

/// Individual hApp score from Bridge
#[derive(Debug, Clone, Deserialize, Default)]
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

// ============================================================================
// Bridge Types
// ============================================================================

/// Available hApp identifiers in the Mycelix ecosystem
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HappId {
    Governance,
    Finance,
    Identity,
    Knowledge,
    Property,
    Energy,
    Media,
    Justice,
    Mail,
    Marketplace,
    Praxis,
    Supplychain,
}

impl HappId {
    /// Get the string representation for zome calls
    pub fn as_str(&self) -> &'static str {
        match self {
            HappId::Governance => "governance",
            HappId::Finance => "finance",
            HappId::Identity => "identity",
            HappId::Knowledge => "knowledge",
            HappId::Property => "property",
            HappId::Energy => "energy",
            HappId::Media => "media",
            HappId::Justice => "justice",
            HappId::Mail => "mail",
            HappId::Marketplace => "marketplace",
            HappId::Praxis => "praxis",
            HappId::Supplychain => "supplychain",
        }
    }

    /// Get default weight for reputation aggregation
    pub fn default_weight(&self) -> f64 {
        match self {
            HappId::Identity => 1.5,   // Identity verification weighs more
            HappId::Justice => 1.3,    // Justice history weighs more
            HappId::Finance => 1.2,
            HappId::Governance => 1.1,
            HappId::Property => 1.0,
            HappId::Energy => 0.9,
            HappId::Knowledge => 0.8,
            HappId::Media => 0.7,
            HappId::Mail => 0.9,
            HappId::Marketplace => 1.0,
            HappId::Praxis => 0.8,
            HappId::Supplychain => 1.0,
        }
    }
}

/// Reputation score from a specific hApp
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HappReputationScore {
    pub happ: HappId,
    pub score: f64,
    pub weight: f64,
    pub last_update: i64,
    pub interaction_count: u64,
}

/// Cross-hApp identity information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrossHappIdentity {
    pub did: String,
    pub verified: bool,
    pub verification_level: u8,
    pub registered_happs: Vec<HappId>,
    pub first_seen: i64,
    pub last_active: i64,
}

/// Aggregated cross-hApp reputation response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrossHappReputation {
    pub did: String,
    pub scores: Vec<HappReputationScore>,
    pub aggregate_score: f64,
    pub confidence: f64,
    pub is_byzantine: bool,
    pub queried_at: i64,
}

impl Default for CrossHappReputation {
    fn default() -> Self {
        Self {
            did: String::new(),
            scores: Vec::new(),
            aggregate_score: 0.5,
            confidence: 0.0,
            is_byzantine: false,
            queried_at: chrono::Utc::now().timestamp_millis(),
        }
    }
}

/// Report type for reputation updates
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReputationReportType {
    PositiveInteraction,
    NegativeInteraction,
    Spam,
    TrustVouch,
    TrustRevoke,
}

/// Reputation report to send to Bridge
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReputationReport {
    pub subject_did: String,
    pub report_type: ReputationReportType,
    pub context: String,
    pub weight: f64,
}

// ============================================================================
// Bridge Client
// ============================================================================

/// Bridge connection state
#[derive(Debug, Clone, PartialEq)]
pub enum BridgeConnectionState {
    /// Not connected
    Disconnected,
    /// Connected in stub mode (no real Bridge)
    StubMode,
    /// Connected to real Bridge zome
    Connected,
    /// Connection failed, using fallback
    Fallback,
}

/// Cached identity information
struct CachedIdentity {
    identity: CrossHappIdentity,
    cached_at: Instant,
}

/// Cached reputation information
struct CachedReputation {
    reputation: CrossHappReputation,
    cached_at: Instant,
}

/// Bridge Client for cross-hApp communication
///
/// Provides:
/// - Identity queries from the Identity hApp
/// - Cross-hApp reputation queries
/// - Reputation reporting for trust updates
/// - Graceful fallback when Bridge is unavailable
///
/// ## Integration with HolochainService
///
/// When connected to a real conductor, this client uses HolochainService
/// to make zome calls to the Bridge coordinator zome for cross-hApp reputation.
pub struct BridgeClient {
    config: Config,
    state: Arc<RwLock<BridgeConnectionState>>,
    /// Cache for identity lookups
    identity_cache: Arc<RwLock<HashMap<String, CachedIdentity>>>,
    /// Cache for reputation lookups
    reputation_cache: Arc<RwLock<HashMap<String, CachedReputation>>>,
    /// TTL for cached data
    cache_ttl: Duration,
    /// Pending reputation reports (for batch submission)
    pending_reports: Arc<RwLock<Vec<ReputationReport>>>,
    /// Reference to HolochainService for zome calls (set after connection)
    holochain: Arc<RwLock<Option<Arc<crate::services::holochain::HolochainService>>>>,
}

impl BridgeClient {
    /// Create a new Bridge client
    pub fn new(config: Config) -> Self {
        let cache_ttl = Duration::from_secs(config.bridge_cache_ttl_secs);

        Self {
            config,
            state: Arc::new(RwLock::new(BridgeConnectionState::Disconnected)),
            identity_cache: Arc::new(RwLock::new(HashMap::new())),
            reputation_cache: Arc::new(RwLock::new(HashMap::new())),
            cache_ttl,
            pending_reports: Arc::new(RwLock::new(Vec::new())),
            holochain: Arc::new(RwLock::new(None)),
        }
    }

    /// Set the HolochainService reference for making Bridge zome calls
    ///
    /// This should be called after the HolochainService is connected to enable
    /// real Bridge zome calls instead of stub data.
    pub async fn set_holochain(&self, holochain: Arc<crate::services::holochain::HolochainService>) {
        let mut hc = self.holochain.write().await;
        *hc = Some(holochain);
        tracing::debug!("BridgeClient: HolochainService reference set");
    }

    /// Check if HolochainService is available for Bridge calls
    pub async fn has_holochain(&self) -> bool {
        self.holochain.read().await.is_some()
    }

    /// Initialize the Bridge connection
    pub async fn connect(&self) -> AppResult<()> {
        let mut state = self.state.write().await;

        // Check if stub mode is enabled
        if self.config.bridge_stub_mode {
            tracing::info!("Bridge client initialized in STUB MODE");
            *state = BridgeConnectionState::StubMode;
            return Ok(());
        }

        // Check if HolochainService is available
        let has_hc = self.holochain.read().await.is_some();
        if has_hc {
            // We have a HolochainService, so we can make real Bridge zome calls
            tracing::info!("Bridge client connected via HolochainService");
            *state = BridgeConnectionState::Connected;
            return Ok(());
        }

        // Fallback: no Holochain connection
        if self.config.bridge_url.is_some() {
            tracing::info!("Bridge URL configured but no HolochainService - using fallback");
            *state = BridgeConnectionState::Fallback;
        } else {
            tracing::info!("No Bridge connection available, using fallback mode");
            *state = BridgeConnectionState::Fallback;
        }

        Ok(())
    }

    /// Check if connected
    pub async fn is_connected(&self) -> bool {
        let state = self.state.read().await;
        matches!(*state, BridgeConnectionState::Connected | BridgeConnectionState::StubMode)
    }

    /// Check if in fallback mode
    pub async fn is_fallback(&self) -> bool {
        let state = self.state.read().await;
        matches!(*state, BridgeConnectionState::Fallback)
    }

    /// Get connection state
    pub async fn connection_state(&self) -> BridgeConnectionState {
        self.state.read().await.clone()
    }

    // =========================================================================
    // Identity Operations
    // =========================================================================

    /// Query identity information for a DID
    ///
    /// First checks cache, then queries Bridge if not found.
    /// Falls back to minimal identity if Bridge unavailable.
    pub async fn query_identity(&self, did: &str) -> AppResult<CrossHappIdentity> {
        // Check cache first
        {
            let cache = self.identity_cache.read().await;
            if let Some(cached) = cache.get(did) {
                if cached.cached_at.elapsed() < self.cache_ttl {
                    tracing::debug!("Bridge identity cache hit for {}", did);
                    return Ok(cached.identity.clone());
                }
            }
        }

        tracing::debug!("Bridge identity cache miss for {}", did);

        let state = self.state.read().await;
        let identity = match *state {
            BridgeConnectionState::Connected => {
                self.fetch_identity_from_bridge(did).await?
            }
            BridgeConnectionState::StubMode => {
                self.stub_identity(did)
            }
            BridgeConnectionState::Fallback | BridgeConnectionState::Disconnected => {
                self.fallback_identity(did)
            }
        };

        // Update cache
        {
            let mut cache = self.identity_cache.write().await;
            cache.insert(did.to_string(), CachedIdentity {
                identity: identity.clone(),
                cached_at: Instant::now(),
            });
        }

        Ok(identity)
    }

    /// Fetch identity from Bridge zome (production)
    ///
    /// Makes a real zome call to the Bridge coordinator to query identity information.
    async fn fetch_identity_from_bridge(&self, did: &str) -> AppResult<CrossHappIdentity> {
        let hc_guard = self.holochain.read().await;
        let hc = match hc_guard.as_ref() {
            Some(hc) => hc.clone(),
            None => {
                tracing::warn!("Bridge: No HolochainService available, using stub identity for {}", did);
                return Ok(self.stub_identity(did));
            }
        };
        drop(hc_guard);

        // Query reputation records to infer identity information
        // The Bridge zome aggregates reputation which tells us which hApps the agent is active in
        match hc.bridge_aggregate_reputation(did).await {
            Ok(rep) => {
                // Extract registered hApps from the scores
                let registered_happs: Vec<HappId> = rep.scores
                    .iter()
                    .filter_map(|s| self.happ_id_from_string(&s.happ_id))
                    .collect();

                // Determine verification level based on hApp count and reputation
                let verification_level = if rep.aggregate >= 0.8 && registered_happs.len() >= 3 {
                    3
                } else if rep.aggregate >= 0.6 && !registered_happs.is_empty() {
                    2
                } else if !registered_happs.is_empty() {
                    1
                } else {
                    0
                };

                Ok(CrossHappIdentity {
                    did: did.to_string(),
                    verified: rep.aggregate >= 0.5 && rep.total_interactions > 0,
                    verification_level,
                    registered_happs,
                    first_seen: chrono::Utc::now().timestamp_millis() - 86400000, // TODO: get from Bridge
                    last_active: chrono::Utc::now().timestamp_millis(),
                })
            }
            Err(e) => {
                tracing::warn!("Bridge: Failed to fetch identity for {}: {}", did, e);
                Ok(self.fallback_identity(did))
            }
        }
    }

    /// Convert hApp ID string to HappId enum
    fn happ_id_from_string(&self, id: &str) -> Option<HappId> {
        match id.to_lowercase().as_str() {
            "governance" => Some(HappId::Governance),
            "finance" => Some(HappId::Finance),
            "identity" => Some(HappId::Identity),
            "knowledge" => Some(HappId::Knowledge),
            "property" => Some(HappId::Property),
            "energy" => Some(HappId::Energy),
            "media" => Some(HappId::Media),
            "justice" => Some(HappId::Justice),
            "mail" => Some(HappId::Mail),
            "marketplace" => Some(HappId::Marketplace),
            "praxis" => Some(HappId::Praxis),
            "supplychain" => Some(HappId::Supplychain),
            _ => None,
        }
    }

    /// Generate stub identity for testing
    fn stub_identity(&self, did: &str) -> CrossHappIdentity {
        CrossHappIdentity {
            did: did.to_string(),
            verified: true,
            verification_level: 1,
            registered_happs: vec![HappId::Mail, HappId::Identity],
            first_seen: chrono::Utc::now().timestamp_millis() - 86400000, // 1 day ago
            last_active: chrono::Utc::now().timestamp_millis(),
        }
    }

    /// Generate fallback identity when Bridge unavailable
    fn fallback_identity(&self, did: &str) -> CrossHappIdentity {
        CrossHappIdentity {
            did: did.to_string(),
            verified: false,
            verification_level: 0,
            registered_happs: vec![],
            first_seen: chrono::Utc::now().timestamp_millis(),
            last_active: chrono::Utc::now().timestamp_millis(),
        }
    }

    // =========================================================================
    // Reputation Operations
    // =========================================================================

    /// Get cross-hApp reputation for a DID
    ///
    /// Aggregates reputation scores from multiple hApps.
    /// Uses weighted average based on hApp importance.
    pub async fn get_reputation(&self, did: &str) -> AppResult<CrossHappReputation> {
        self.get_reputation_with_context(did, &[
            HappId::Identity,
            HappId::Mail,
            HappId::Finance,
            HappId::Justice,
            HappId::Governance,
        ]).await
    }

    /// Get cross-hApp reputation with specific context hApps
    pub async fn get_reputation_with_context(
        &self,
        did: &str,
        context_happs: &[HappId],
    ) -> AppResult<CrossHappReputation> {
        // Check cache first
        {
            let cache = self.reputation_cache.read().await;
            if let Some(cached) = cache.get(did) {
                if cached.cached_at.elapsed() < self.cache_ttl {
                    tracing::debug!("Bridge reputation cache hit for {}", did);
                    return Ok(cached.reputation.clone());
                }
            }
        }

        tracing::debug!("Bridge reputation cache miss for {}", did);

        let state = self.state.read().await;
        let reputation = match *state {
            BridgeConnectionState::Connected => {
                self.fetch_reputation_from_bridge(did, context_happs).await?
            }
            BridgeConnectionState::StubMode => {
                self.stub_reputation(did, context_happs)
            }
            BridgeConnectionState::Fallback | BridgeConnectionState::Disconnected => {
                self.fallback_reputation(did)
            }
        };

        // Update cache
        {
            let mut cache = self.reputation_cache.write().await;
            cache.insert(did.to_string(), CachedReputation {
                reputation: reputation.clone(),
                cached_at: Instant::now(),
            });
        }

        Ok(reputation)
    }

    /// Fetch reputation from Bridge zome (production)
    ///
    /// Makes a real zome call to the Bridge coordinator to get cross-hApp reputation.
    async fn fetch_reputation_from_bridge(
        &self,
        did: &str,
        context_happs: &[HappId],
    ) -> AppResult<CrossHappReputation> {
        let hc_guard = self.holochain.read().await;
        let hc = match hc_guard.as_ref() {
            Some(hc) => hc.clone(),
            None => {
                tracing::warn!("Bridge: No HolochainService available, using stub reputation for {}", did);
                return Ok(self.stub_reputation(did, context_happs));
            }
        };
        drop(hc_guard);

        let now = chrono::Utc::now().timestamp_millis();

        // Call Bridge zome to get aggregate reputation
        match hc.bridge_aggregate_reputation(did).await {
            Ok(bridge_rep) => {
                // Convert Bridge zome response to our internal format
                let scores: Vec<HappReputationScore> = bridge_rep.scores
                    .iter()
                    .filter_map(|s| {
                        let happ_id = self.happ_id_from_string(&s.happ_id)?;
                        // Only include if in context_happs or if context_happs is empty
                        if !context_happs.is_empty() && !context_happs.contains(&happ_id) {
                            return None;
                        }
                        Some(HappReputationScore {
                            happ: happ_id,
                            score: s.score,
                            weight: happ_id.default_weight(),
                            last_update: s.last_updated as i64,
                            interaction_count: s.interactions,
                        })
                    })
                    .collect();

                // Calculate aggregate from filtered scores
                let aggregate = if scores.is_empty() {
                    bridge_rep.aggregate
                } else {
                    self.calculate_aggregate(&scores)
                };

                // Calculate confidence based on interaction count and hApp diversity
                let confidence = if bridge_rep.total_interactions == 0 {
                    0.0
                } else {
                    let interaction_factor = (bridge_rep.total_interactions as f64 / 100.0).min(1.0);
                    let diversity_factor = (scores.len() as f64 / 5.0).min(1.0);
                    interaction_factor * 0.7 + diversity_factor * 0.3
                };

                // Check if trustworthy (byzantine detection)
                let is_trustworthy = hc.bridge_is_trustworthy(did, 0.3).await.unwrap_or(true);

                Ok(CrossHappReputation {
                    did: did.to_string(),
                    scores,
                    aggregate_score: aggregate,
                    confidence,
                    is_byzantine: !is_trustworthy,
                    queried_at: now,
                })
            }
            Err(e) => {
                tracing::warn!("Bridge: Failed to fetch reputation for {}: {}", did, e);
                Ok(self.fallback_reputation(did))
            }
        }
    }

    /// Generate stub reputation for testing
    fn stub_reputation(&self, did: &str, context_happs: &[HappId]) -> CrossHappReputation {
        let now = chrono::Utc::now().timestamp_millis();
        let scores: Vec<HappReputationScore> = context_happs
            .iter()
            .map(|happ| HappReputationScore {
                happ: *happ,
                score: 0.5, // Neutral default
                weight: happ.default_weight(),
                last_update: now,
                interaction_count: 0,
            })
            .collect();

        let aggregate = self.calculate_aggregate(&scores);

        CrossHappReputation {
            did: did.to_string(),
            scores,
            aggregate_score: aggregate,
            confidence: 0.5,
            is_byzantine: false,
            queried_at: now,
        }
    }

    /// Generate fallback reputation when Bridge unavailable
    fn fallback_reputation(&self, did: &str) -> CrossHappReputation {
        CrossHappReputation {
            did: did.to_string(),
            scores: vec![],
            aggregate_score: self.config.bridge_fallback_trust,
            confidence: 0.0,
            is_byzantine: false,
            queried_at: chrono::Utc::now().timestamp_millis(),
        }
    }

    /// Calculate weighted aggregate from scores
    fn calculate_aggregate(&self, scores: &[HappReputationScore]) -> f64 {
        if scores.is_empty() {
            return 0.5;
        }

        let total_weight: f64 = scores.iter().map(|s| s.weight).sum();
        if total_weight == 0.0 {
            return 0.5;
        }

        let weighted_sum: f64 = scores.iter().map(|s| s.score * s.weight).sum();
        weighted_sum / total_weight
    }

    // =========================================================================
    // Reputation Reporting
    // =========================================================================

    /// Report a positive interaction with a DID
    ///
    /// This contributes to improving the DID's reputation across the ecosystem.
    pub async fn report_positive_interaction(&self, did: &str, context: &str) -> AppResult<()> {
        self.report_reputation(ReputationReport {
            subject_did: did.to_string(),
            report_type: ReputationReportType::PositiveInteraction,
            context: context.to_string(),
            weight: 1.0,
        }).await
    }

    /// Report a negative interaction with a DID
    ///
    /// This contributes to lowering the DID's reputation.
    pub async fn report_negative_interaction(&self, did: &str, context: &str) -> AppResult<()> {
        self.report_reputation(ReputationReport {
            subject_did: did.to_string(),
            report_type: ReputationReportType::NegativeInteraction,
            context: context.to_string(),
            weight: 1.0,
        }).await
    }

    /// Report spam from a DID
    ///
    /// This has a stronger negative impact than a regular negative interaction.
    pub async fn report_spam(&self, did: &str, context: &str) -> AppResult<()> {
        self.report_reputation(ReputationReport {
            subject_did: did.to_string(),
            report_type: ReputationReportType::Spam,
            context: context.to_string(),
            weight: 2.0, // Higher weight for spam
        }).await
    }

    /// Vouch for a DID (explicit trust)
    pub async fn vouch_for(&self, did: &str, context: &str) -> AppResult<()> {
        self.report_reputation(ReputationReport {
            subject_did: did.to_string(),
            report_type: ReputationReportType::TrustVouch,
            context: context.to_string(),
            weight: 1.5,
        }).await
    }

    /// Submit a reputation report
    async fn report_reputation(&self, report: ReputationReport) -> AppResult<()> {
        // Save the DID before any potential move of the report
        let subject_did = report.subject_did.clone();

        let state = self.state.read().await;

        match *state {
            BridgeConnectionState::Connected => {
                self.submit_report_to_bridge(&report).await?;
            }
            BridgeConnectionState::StubMode => {
                tracing::debug!(
                    "STUB: Reputation report for {} ({:?})",
                    report.subject_did,
                    report.report_type
                );
            }
            BridgeConnectionState::Fallback | BridgeConnectionState::Disconnected => {
                // Queue for later submission
                let mut pending = self.pending_reports.write().await;
                pending.push(report);
                tracing::debug!("Queued reputation report for later submission ({} pending)", pending.len());
            }
        }

        // Invalidate cache for this DID
        {
            let mut cache = self.reputation_cache.write().await;
            cache.remove(&subject_did);
        }

        Ok(())
    }

    /// Submit report to Bridge zome (production)
    ///
    /// Records a reputation event via the Bridge coordinator zome.
    async fn submit_report_to_bridge(&self, report: &ReputationReport) -> AppResult<()> {
        let hc_guard = self.holochain.read().await;
        let hc = match hc_guard.as_ref() {
            Some(hc) => hc.clone(),
            None => {
                tracing::warn!("Bridge: No HolochainService available, cannot submit report for {}", report.subject_did);
                return Err(AppError::TrustUnavailable("Bridge zome not connected".into()));
            }
        };
        drop(hc_guard);

        // Convert report type to score delta and interaction counts
        let (score_delta, is_positive) = match report.report_type {
            ReputationReportType::PositiveInteraction => (0.05 * report.weight, true),
            ReputationReportType::NegativeInteraction => (-0.05 * report.weight, false),
            ReputationReportType::Spam => (-0.15 * report.weight, false),
            ReputationReportType::TrustVouch => (0.1 * report.weight, true),
            ReputationReportType::TrustRevoke => (-0.1 * report.weight, false),
        };

        // First, query existing reputation to update it
        let existing = hc.bridge_query_reputation(&report.subject_did, Some("mail")).await
            .unwrap_or_default();

        let (current_score, interactions, negative_interactions) = if let Some(rec) = existing.first() {
            (rec.score, rec.interactions, rec.negative_interactions)
        } else {
            (0.5, 0, 0) // Default neutral score
        };

        // Calculate new score (bounded 0.0 - 1.0)
        let new_score = (current_score + score_delta).clamp(0.0, 1.0);
        let new_interactions = if is_positive { interactions + 1 } else { interactions };
        let new_negative = if !is_positive { negative_interactions + 1 } else { negative_interactions };

        // Submit to Bridge zome
        let input = crate::services::holochain::BridgeRecordReputationInput {
            agent: report.subject_did.clone(),
            happ_id: "mail".to_string(),
            happ_name: "Mycelix Mail".to_string(),
            score: new_score,
            interactions: new_interactions,
            negative_interactions: new_negative,
            evidence_hash: Some(format!("mail:{}", report.context)),
        };

        match hc.bridge_record_reputation(input).await {
            Ok(_) => {
                tracing::debug!(
                    "Bridge: Recorded reputation for {} ({:?}, new_score={})",
                    report.subject_did,
                    report.report_type,
                    new_score
                );
                Ok(())
            }
            Err(e) => {
                tracing::warn!("Bridge: Failed to record reputation for {}: {}", report.subject_did, e);
                Err(e)
            }
        }
    }

    /// Flush pending reports when Bridge becomes available
    pub async fn flush_pending_reports(&self) -> AppResult<usize> {
        let state = self.state.read().await;
        if !matches!(*state, BridgeConnectionState::Connected) {
            return Ok(0);
        }
        drop(state);

        // Take all pending reports out of the queue
        let reports: Vec<ReputationReport> = {
            let mut pending = self.pending_reports.write().await;
            std::mem::take(&mut *pending)
        };

        let count = reports.len();
        let mut failed = Vec::new();

        // Process each report
        for report in reports {
            if let Err(e) = self.submit_report_to_bridge(&report).await {
                tracing::warn!("Failed to submit pending report: {}", e);
                // Collect failed reports for re-queueing
                failed.push(report);
            }
        }

        // Re-queue failed reports
        if !failed.is_empty() {
            let mut pending = self.pending_reports.write().await;
            pending.extend(failed.into_iter());
        }

        Ok(count)
    }

    // =========================================================================
    // Cache Management
    // =========================================================================

    /// Invalidate cache for a specific DID
    pub async fn invalidate_cache(&self, did: &str) {
        {
            let mut cache = self.identity_cache.write().await;
            cache.remove(did);
        }
        {
            let mut cache = self.reputation_cache.write().await;
            cache.remove(did);
        }
        tracing::debug!("Invalidated Bridge cache for {}", did);
    }

    /// Clear all caches
    pub async fn clear_cache(&self) {
        {
            let mut cache = self.identity_cache.write().await;
            cache.clear();
        }
        {
            let mut cache = self.reputation_cache.write().await;
            cache.clear();
        }
        tracing::info!("Cleared all Bridge caches");
    }

    /// Get cache statistics
    pub async fn cache_stats(&self) -> BridgeCacheStats {
        let identity_count = self.identity_cache.read().await.len();
        let reputation_count = self.reputation_cache.read().await.len();
        let pending_reports = self.pending_reports.read().await.len();

        BridgeCacheStats {
            identity_entries: identity_count,
            reputation_entries: reputation_count,
            pending_reports,
            cache_ttl_secs: self.cache_ttl.as_secs(),
        }
    }
}

/// Bridge cache statistics
#[derive(Debug, Clone, Serialize)]
pub struct BridgeCacheStats {
    pub identity_entries: usize,
    pub reputation_entries: usize,
    pub pending_reports: usize,
    pub cache_ttl_secs: u64,
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Check if a DID has sufficient reputation for receiving mail
pub fn has_sufficient_reputation(reputation: &CrossHappReputation, min_trust: f64) -> bool {
    reputation.aggregate_score >= min_trust && !reputation.is_byzantine
}

/// Determine spam likelihood based on reputation
pub fn spam_likelihood(reputation: &CrossHappReputation) -> f64 {
    if reputation.is_byzantine {
        return 1.0;
    }

    // Lower reputation = higher spam likelihood
    1.0 - reputation.aggregate_score.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

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
    async fn test_stub_mode() {
        let config = test_config();
        let bridge = BridgeClient::new(config);
        bridge.connect().await.unwrap();

        assert!(bridge.is_connected().await);
    }

    #[tokio::test]
    async fn test_query_identity() {
        let config = test_config();
        let bridge = BridgeClient::new(config);
        bridge.connect().await.unwrap();

        let identity = bridge.query_identity("did:mycelix:test").await.unwrap();
        assert_eq!(identity.did, "did:mycelix:test");
        assert!(identity.verified);
    }

    #[tokio::test]
    async fn test_get_reputation() {
        let config = test_config();
        let bridge = BridgeClient::new(config);
        bridge.connect().await.unwrap();

        let reputation = bridge.get_reputation("did:mycelix:test").await.unwrap();
        assert_eq!(reputation.did, "did:mycelix:test");
        assert_eq!(reputation.aggregate_score, 0.5);
    }

    #[tokio::test]
    async fn test_cache_hit() {
        let config = test_config();
        let bridge = BridgeClient::new(config);
        bridge.connect().await.unwrap();

        // First call
        let _ = bridge.get_reputation("did:mycelix:test").await.unwrap();

        // Second call should hit cache
        let stats_before = bridge.cache_stats().await;
        let _ = bridge.get_reputation("did:mycelix:test").await.unwrap();
        let stats_after = bridge.cache_stats().await;

        assert_eq!(stats_before.reputation_entries, stats_after.reputation_entries);
    }

    #[test]
    fn test_sufficient_reputation() {
        let reputation = CrossHappReputation {
            did: "test".to_string(),
            scores: vec![],
            aggregate_score: 0.5,
            confidence: 0.8,
            is_byzantine: false,
            queried_at: 0,
        };

        assert!(has_sufficient_reputation(&reputation, 0.3));
        assert!(!has_sufficient_reputation(&reputation, 0.6));
    }

    #[test]
    fn test_spam_likelihood() {
        let good_reputation = CrossHappReputation {
            did: "good".to_string(),
            scores: vec![],
            aggregate_score: 0.9,
            confidence: 0.8,
            is_byzantine: false,
            queried_at: 0,
        };

        let bad_reputation = CrossHappReputation {
            did: "bad".to_string(),
            scores: vec![],
            aggregate_score: 0.1,
            confidence: 0.8,
            is_byzantine: false,
            queried_at: 0,
        };

        assert!(spam_likelihood(&good_reputation) < 0.2);
        assert!(spam_likelihood(&bad_reputation) > 0.8);
    }
}
