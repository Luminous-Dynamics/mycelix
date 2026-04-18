// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Storage service for message body content
//!
//! Provides content-addressed storage for email bodies using IPFS.
//! Bodies are encrypted before storage and decrypted on retrieval.

use reqwest::multipart::{Form, Part};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::config::Config;
use crate::error::{AppError, AppResult};

/// IPFS API response for add operation
#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
struct IpfsAddResponse {
    /// Content identifier (CID)
    hash: String,
    /// Name of the uploaded file
    #[allow(dead_code)]
    name: String,
    /// Size in bytes
    #[allow(dead_code)]
    size: String,
}

/// Stored content metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StoredContent {
    /// IPFS CID (Content Identifier)
    pub cid: String,
    /// Size in bytes
    pub size: usize,
    /// Whether the content is encrypted
    pub encrypted: bool,
}

/// Storage service for content-addressed storage
pub struct StorageService {
    /// HTTP client
    client: reqwest::Client,
    /// IPFS API URL
    ipfs_url: String,
    /// Whether IPFS is available
    ipfs_available: bool,
    /// Local fallback storage (in-memory for development)
    local_store: Arc<tokio::sync::RwLock<std::collections::HashMap<String, Vec<u8>>>>,
}

impl StorageService {
    /// Create a new storage service
    pub fn new(_config: &Config) -> Self {
        let ipfs_url = std::env::var("IPFS_API_URL")
            .unwrap_or_else(|_| "http://localhost:5001".to_string());

        Self {
            client: reqwest::Client::new(),
            ipfs_url,
            ipfs_available: false, // Will be set on first use
            local_store: Arc::new(tokio::sync::RwLock::new(std::collections::HashMap::new())),
        }
    }

    /// Check if IPFS is available
    pub async fn check_ipfs(&mut self) -> bool {
        match self
            .client
            .post(format!("{}/api/v0/id", self.ipfs_url))
            .send()
            .await
        {
            Ok(resp) if resp.status().is_success() => {
                self.ipfs_available = true;
                tracing::info!("IPFS node connected at {}", self.ipfs_url);
                true
            }
            _ => {
                self.ipfs_available = false;
                tracing::warn!(
                    "IPFS not available at {}. Using local fallback storage.",
                    self.ipfs_url
                );
                false
            }
        }
    }

    /// Store content and return its CID
    ///
    /// If IPFS is available, stores to IPFS.
    /// Otherwise, stores to local in-memory store with a computed hash.
    pub async fn store(&self, content: &[u8]) -> AppResult<StoredContent> {
        if self.ipfs_available {
            self.store_ipfs(content).await
        } else {
            self.store_local(content).await
        }
    }

    /// Store content to IPFS
    async fn store_ipfs(&self, content: &[u8]) -> AppResult<StoredContent> {
        let form = Form::new().part(
            "file",
            Part::bytes(content.to_vec()).file_name("body.bin"),
        );

        let response = self
            .client
            .post(format!("{}/api/v0/add", self.ipfs_url))
            .multipart(form)
            .send()
            .await
            .map_err(|e| AppError::InternalError(format!("IPFS add failed: {}", e)))?;

        if !response.status().is_success() {
            return Err(AppError::InternalError(format!(
                "IPFS returned error: {}",
                response.status()
            )));
        }

        let ipfs_response: IpfsAddResponse = response
            .json()
            .await
            .map_err(|e| AppError::InternalError(format!("Invalid IPFS response: {}", e)))?;

        Ok(StoredContent {
            cid: ipfs_response.hash,
            size: content.len(),
            encrypted: true, // Assuming content is encrypted before storage
        })
    }

    /// Store content to local fallback storage
    async fn store_local(&self, content: &[u8]) -> AppResult<StoredContent> {
        use sha2::{Digest, Sha256};

        // Compute a CID-like hash
        let mut hasher = Sha256::new();
        hasher.update(content);
        let hash = hasher.finalize();

        // Create a fake CID with Qm prefix (like IPFS CIDv0)
        use base64::{engine::general_purpose::URL_SAFE_NO_PAD, Engine as _};
        let cid = format!("Qm{}", URL_SAFE_NO_PAD.encode(&hash[..24]));

        // Store locally
        let mut store = self.local_store.write().await;
        store.insert(cid.clone(), content.to_vec());

        tracing::debug!("Stored {} bytes locally with CID: {}", content.len(), cid);

        Ok(StoredContent {
            cid,
            size: content.len(),
            encrypted: true,
        })
    }

    /// Retrieve content by CID
    pub async fn retrieve(&self, cid: &str) -> AppResult<Vec<u8>> {
        if self.ipfs_available {
            self.retrieve_ipfs(cid).await
        } else {
            self.retrieve_local(cid).await
        }
    }

    /// Retrieve content from IPFS
    async fn retrieve_ipfs(&self, cid: &str) -> AppResult<Vec<u8>> {
        let response = self
            .client
            .post(format!("{}/api/v0/cat", self.ipfs_url))
            .query(&[("arg", cid)])
            .send()
            .await
            .map_err(|e| AppError::InternalError(format!("IPFS cat failed: {}", e)))?;

        if !response.status().is_success() {
            return Err(AppError::NotFound(format!("Content not found: {}", cid)));
        }

        let bytes = response
            .bytes()
            .await
            .map_err(|e| AppError::InternalError(format!("Failed to read IPFS content: {}", e)))?;

        Ok(bytes.to_vec())
    }

    /// Retrieve content from local storage
    async fn retrieve_local(&self, cid: &str) -> AppResult<Vec<u8>> {
        let store = self.local_store.read().await;
        store
            .get(cid)
            .cloned()
            .ok_or_else(|| AppError::NotFound(format!("Content not found: {}", cid)))
    }

    /// Check if content exists
    pub async fn exists(&self, cid: &str) -> bool {
        if self.ipfs_available {
            // Try to stat the object
            self.client
                .post(format!("{}/api/v0/files/stat", self.ipfs_url))
                .query(&[("arg", format!("/ipfs/{}", cid))])
                .send()
                .await
                .map(|r| r.status().is_success())
                .unwrap_or(false)
        } else {
            self.local_store.read().await.contains_key(cid)
        }
    }

    /// Pin content (prevent garbage collection)
    pub async fn pin(&self, cid: &str) -> AppResult<()> {
        if !self.ipfs_available {
            // Local storage doesn't need pinning
            return Ok(());
        }

        let response = self
            .client
            .post(format!("{}/api/v0/pin/add", self.ipfs_url))
            .query(&[("arg", cid)])
            .send()
            .await
            .map_err(|e| AppError::InternalError(format!("IPFS pin failed: {}", e)))?;

        if !response.status().is_success() {
            return Err(AppError::InternalError(format!(
                "Failed to pin content: {}",
                cid
            )));
        }

        Ok(())
    }
}

/// Combined storage: encrypts content then stores
pub async fn store_encrypted(
    storage: &StorageService,
    plaintext: &[u8],
    encryption_key: &[u8; 32],
) -> AppResult<StoredContent> {
    use super::crypto::encrypt_symmetric;

    let (ciphertext, nonce) = encrypt_symmetric(plaintext, encryption_key)?;

    // Prepend nonce to ciphertext for storage
    let mut data = nonce.to_vec();
    data.extend(ciphertext);

    storage.store(&data).await
}

/// Combined retrieval: fetches and decrypts content
pub async fn retrieve_decrypted(
    storage: &StorageService,
    cid: &str,
    encryption_key: &[u8; 32],
) -> AppResult<Vec<u8>> {
    use super::crypto::decrypt_symmetric;

    let data = storage.retrieve(cid).await?;

    if data.len() < 12 {
        return Err(AppError::EncryptionError(
            "Invalid stored content: too short".to_string(),
        ));
    }

    // Extract nonce (first 12 bytes) and ciphertext
    let mut nonce = [0u8; 12];
    nonce.copy_from_slice(&data[..12]);
    let ciphertext = &data[12..];

    decrypt_symmetric(ciphertext, &nonce, encryption_key)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_local_storage_roundtrip() {
        let config = Config::from_env().unwrap_or_else(|_| {
            // Create minimal config for testing
            std::env::set_var("JWT_SECRET", "test-secret-for-testing");
            Config::from_env().unwrap()
        });

        let storage = StorageService::new(&config);
        let content = b"Hello, Mycelix storage!";

        // Store
        let stored = storage.store(content).await.unwrap();
        assert!(!stored.cid.is_empty());
        assert_eq!(stored.size, content.len());

        // Retrieve
        let retrieved = storage.retrieve(&stored.cid).await.unwrap();
        assert_eq!(retrieved, content);
    }

    #[tokio::test]
    async fn test_encrypted_storage_roundtrip() {
        let config = Config::from_env().unwrap_or_else(|_| {
            std::env::set_var("JWT_SECRET", "test-secret-for-testing");
            Config::from_env().unwrap()
        });

        let storage = StorageService::new(&config);
        let plaintext = b"Secret message body";
        let key = [42u8; 32];

        // Store encrypted
        let stored = store_encrypted(&storage, plaintext, &key).await.unwrap();
        assert!(!stored.cid.is_empty());

        // Retrieve and decrypt
        let decrypted = retrieve_decrypted(&storage, &stored.cid, &key)
            .await
            .unwrap();
        assert_eq!(decrypted, plaintext);
    }

    #[tokio::test]
    async fn test_not_found() {
        let config = Config::from_env().unwrap_or_else(|_| {
            std::env::set_var("JWT_SECRET", "test-secret-for-testing");
            Config::from_env().unwrap()
        });

        let storage = StorageService::new(&config);
        let result = storage.retrieve("QmNonExistentCid").await;
        assert!(result.is_err());
    }
}
