// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! IPFS Service - Decentralized storage integration
//!
//! Handles uploads to IPFS via Web3.Storage for permanent,
//! content-addressed storage of music files.

use anyhow::Result;

/// IPFS service for file storage
pub struct IpfsService {
    client: ipfs_api_backend_hyper::IpfsClient,
    gateway_url: String,
}

impl IpfsService {
    pub fn new(api_url: &str, gateway_url: &str) -> Result<Self> {
        let client = ipfs_api_backend_hyper::IpfsClient::from_str(api_url)?;
        Ok(Self {
            client,
            gateway_url: gateway_url.to_string(),
        })
    }

    /// Upload data to IPFS
    pub async fn upload(&self, data: Vec<u8>) -> Result<String> {
        let cursor = std::io::Cursor::new(data);
        let response = self.client.add(cursor).await?;
        Ok(response.hash)
    }

    /// Get gateway URL for a hash
    pub fn gateway_url(&self, hash: &str) -> String {
        format!("{}/ipfs/{}", self.gateway_url, hash)
    }

    /// Pin a hash to ensure persistence
    pub async fn pin(&self, hash: &str) -> Result<()> {
        self.client.pin_add(hash, true).await?;
        Ok(())
    }

    /// Check if content exists
    pub async fn exists(&self, hash: &str) -> bool {
        self.client.cat(hash).await.is_ok()
    }
}
