// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Cache Service - Redis caching layer
//!
//! Provides caching for frequently accessed data
//! to reduce database load and improve response times.

use anyhow::Result;
use redis::{AsyncCommands, Client};
use serde::{de::DeserializeOwned, Serialize};

/// Cache service with Redis backend
pub struct CacheService {
    client: Client,
}

impl CacheService {
    pub fn new(redis_url: &str) -> Result<Self> {
        let client = Client::open(redis_url)?;
        Ok(Self { client })
    }

    /// Get a value from cache
    pub async fn get<T: DeserializeOwned>(&self, key: &str) -> Result<Option<T>> {
        let mut conn = self.client.get_multiplexed_async_connection().await?;
        let value: Option<String> = conn.get(key).await?;

        match value {
            Some(json) => Ok(Some(serde_json::from_str(&json)?)),
            None => Ok(None),
        }
    }

    /// Set a value in cache with TTL
    pub async fn set<T: Serialize>(&self, key: &str, value: &T, ttl_seconds: u64) -> Result<()> {
        let mut conn = self.client.get_multiplexed_async_connection().await?;
        let json = serde_json::to_string(value)?;
        conn.set_ex(key, json, ttl_seconds).await?;
        Ok(())
    }

    /// Delete a value from cache
    pub async fn delete(&self, key: &str) -> Result<()> {
        let mut conn = self.client.get_multiplexed_async_connection().await?;
        conn.del(key).await?;
        Ok(())
    }

    /// Check and set a nonce (for replay protection)
    pub async fn check_nonce(&self, nonce: &str, ttl_seconds: u64) -> Result<bool> {
        let mut conn = self.client.get_multiplexed_async_connection().await?;
        let key = format!("nonce:{}", nonce);

        // SETNX returns true if key was set (nonce is new)
        let was_set: bool = conn.set_nx(&key, "1").await?;
        if was_set {
            // Set expiry
            conn.expire(&key, ttl_seconds as i64).await?;
        }
        Ok(was_set)
    }

    /// Increment a counter (for rate limiting)
    pub async fn increment(&self, key: &str, ttl_seconds: u64) -> Result<i64> {
        let mut conn = self.client.get_multiplexed_async_connection().await?;
        let count: i64 = conn.incr(key, 1).await?;
        if count == 1 {
            conn.expire(key, ttl_seconds as i64).await?;
        }
        Ok(count)
    }
}
