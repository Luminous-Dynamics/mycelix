// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Trust score caching service
//!
//! Caches trust scores from the DHT to reduce query load.
//! Uses moka for async-aware LRU caching with TTL expiration.
//!
//! Now enhanced with Bridge integration for cross-hApp reputation.
//! When available, combines local MATL trust with ecosystem-wide reputation.

use std::sync::Arc;
use moka::future::Cache;

use crate::config::Config;
use crate::services::holochain::{HolochainService, MatlTrustResult};
use crate::services::bridge::BridgeClient;
use crate::error::AppResult;

/// Cached trust information
#[derive(Debug, Clone)]
pub struct CachedTrust {
    pub did: String,
    pub score: f64,
    pub is_byzantine: bool,
    pub interaction_count: u64,
    pub is_new_user: bool,
    pub cached_at: std::time::Instant,
    /// Cross-hApp reputation score (if available)
    pub cross_happ_score: Option<f64>,
    /// Combined score (weighted average of local and cross-hApp)
    pub combined_score: f64,
}

impl From<MatlTrustResult> for CachedTrust {
    fn from(result: MatlTrustResult) -> Self {
        Self {
            did: result.did,
            score: result.score,
            is_byzantine: result.is_byzantine,
            interaction_count: result.interaction_count,
            is_new_user: result.is_new_user,
            cached_at: std::time::Instant::now(),
            cross_happ_score: None,
            combined_score: result.score, // Start with local score
        }
    }
}

impl CachedTrust {
    /// Update with cross-hApp reputation and recalculate combined score
    pub fn with_cross_happ_score(mut self, cross_happ_score: f64, local_weight: f64) -> Self {
        self.cross_happ_score = Some(cross_happ_score);
        // Weighted average: local has more weight for new ecosystem, equal weight for mature
        let cross_weight = 1.0 - local_weight;
        self.combined_score = (self.score * local_weight) + (cross_happ_score * cross_weight);
        self
    }
}

/// Trust score cache service
pub struct TrustCacheService {
    cache: Cache<String, CachedTrust>,
    holochain: Arc<HolochainService>,
    bridge: Option<Arc<BridgeClient>>,
    byzantine_threshold: f64,
    /// Weight for local MATL score vs cross-hApp (0.0-1.0)
    local_weight: f64,
}

impl TrustCacheService {
    /// Create a new trust cache service
    pub fn new(config: &Config, holochain: Arc<HolochainService>) -> Self {
        let cache = Cache::builder()
            .max_capacity(config.trust_cache_max_entries)
            .time_to_live(config.trust_cache_ttl())
            .build();

        Self {
            cache,
            holochain,
            bridge: None,
            byzantine_threshold: config.byzantine_threshold,
            local_weight: 0.6, // Default: 60% local, 40% cross-hApp
        }
    }

    /// Create a new trust cache service with Bridge integration
    pub fn new_with_bridge(
        config: &Config,
        holochain: Arc<HolochainService>,
        bridge: Arc<BridgeClient>,
    ) -> Self {
        let cache = Cache::builder()
            .max_capacity(config.trust_cache_max_entries)
            .time_to_live(config.trust_cache_ttl())
            .build();

        Self {
            cache,
            holochain,
            bridge: Some(bridge),
            byzantine_threshold: config.byzantine_threshold,
            local_weight: 0.6, // Default: 60% local, 40% cross-hApp
        }
    }

    /// Set the Bridge client after initialization
    pub fn set_bridge(&mut self, bridge: Arc<BridgeClient>) {
        self.bridge = Some(bridge);
    }

    /// Set the weight for local vs cross-hApp scores
    pub fn set_local_weight(&mut self, weight: f64) {
        self.local_weight = weight.clamp(0.0, 1.0);
    }

    /// Get trust score for a DID, using cache if available
    ///
    /// If Bridge is available, this will also fetch cross-hApp reputation
    /// and compute a combined score.
    pub async fn get_trust(&self, did: &str) -> AppResult<CachedTrust> {
        // Check cache first
        if let Some(cached) = self.cache.get(did).await {
            tracing::debug!("Trust cache hit for {}", did);
            return Ok(cached);
        }

        tracing::debug!("Trust cache miss for {}, fetching from DHT", did);

        // Fetch local MATL trust from Holochain
        let result = self.holochain.evaluate_trust_matl(did).await?;
        let mut cached = CachedTrust::from(result);

        // If Bridge is available, also fetch cross-hApp reputation
        if let Some(ref bridge) = self.bridge {
            if bridge.is_connected().await {
                match bridge.get_reputation(did).await {
                    Ok(cross_happ) => {
                        tracing::debug!(
                            "Got cross-hApp reputation for {}: {}",
                            did,
                            cross_happ.aggregate_score
                        );
                        // If Bridge says Byzantine, trust that
                        if cross_happ.is_byzantine {
                            cached.is_byzantine = true;
                        }
                        // Update with cross-hApp score
                        cached = cached.with_cross_happ_score(
                            cross_happ.aggregate_score,
                            self.local_weight,
                        );
                    }
                    Err(e) => {
                        tracing::warn!(
                            "Failed to get cross-hApp reputation for {}: {}",
                            did, e
                        );
                        // Continue with local-only score
                    }
                }
            }
        }

        // Store in cache
        self.cache.insert(did.to_string(), cached.clone()).await;

        Ok(cached)
    }

    /// Get trust score using only local MATL (no Bridge)
    pub async fn get_local_trust(&self, did: &str) -> AppResult<CachedTrust> {
        let result = self.holochain.evaluate_trust_matl(did).await?;
        Ok(CachedTrust::from(result))
    }

    /// Get trust score value only (returns combined score if Bridge available)
    pub async fn get_score(&self, did: &str) -> AppResult<f64> {
        Ok(self.get_trust(did).await?.combined_score)
    }

    /// Get local-only trust score value
    pub async fn get_local_score(&self, did: &str) -> AppResult<f64> {
        Ok(self.get_trust(did).await?.score)
    }

    /// Check if a DID is flagged as Byzantine
    pub async fn is_byzantine(&self, did: &str) -> AppResult<bool> {
        let trust = self.get_trust(did).await?;
        Ok(trust.is_byzantine || trust.score < self.byzantine_threshold)
    }

    /// Invalidate cache for a specific DID
    pub async fn invalidate(&self, did: &str) {
        self.cache.invalidate(did).await;
        tracing::debug!("Invalidated trust cache for {}", did);
    }

    /// Invalidate entire cache
    pub async fn invalidate_all(&self) {
        self.cache.invalidate_all();
        tracing::info!("Invalidated entire trust cache");
    }

    /// Get cache statistics
    pub fn stats(&self) -> CacheStats {
        CacheStats {
            entry_count: self.cache.entry_count(),
            weighted_size: self.cache.weighted_size(),
        }
    }

    /// Batch fetch trust scores for multiple DIDs
    pub async fn get_trust_batch(&self, dids: &[String]) -> Vec<(String, AppResult<CachedTrust>)> {
        let mut results = Vec::with_capacity(dids.len());

        for did in dids {
            let result = self.get_trust(did).await;
            results.push((did.clone(), result));
        }

        results
    }

    /// Pre-warm cache for known contacts
    pub async fn warm_cache(&self, dids: &[String]) {
        tracing::info!("Warming trust cache for {} DIDs", dids.len());

        for did in dids {
            if self.cache.get(did).await.is_none() {
                if let Ok(trust) = self.holochain.evaluate_trust_matl(did).await {
                    self.cache.insert(did.clone(), CachedTrust::from(trust)).await;
                }
            }
        }

        tracing::info!("Trust cache warmed, {} entries", self.cache.entry_count());
    }
}

/// Cache statistics
#[derive(Debug, Clone)]
pub struct CacheStats {
    pub entry_count: u64,
    pub weighted_size: u64,
}
