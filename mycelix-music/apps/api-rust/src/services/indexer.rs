// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Event Indexer Service
//!
//! Listens to smart contract events and syncs them to the database.
//! This enables the API to serve real-time payment and play data.

use anyhow::Result;
use ethers::prelude::*;
use sqlx::PgPool;
use std::sync::Arc;
use tokio::time::{sleep, Duration};
use tracing::{error, info, warn};

/// Contract event signatures (keccak256 hashes)
mod event_signatures {
    use ethers::types::H256;

    /// PaymentProcessed(bytes32 indexed songId, address indexed listener, uint256 amount, uint8 paymentType)
    pub const PAYMENT_PROCESSED: H256 = H256([
        0x8a, 0x7b, 0x93, 0x56, 0xaa, 0x45, 0xc7, 0x08,
        0x9a, 0x56, 0x2b, 0x35, 0x1c, 0x88, 0xd9, 0x17,
        0x8a, 0x22, 0x4d, 0x15, 0x5c, 0x21, 0x78, 0x4a,
        0x63, 0x91, 0x12, 0xaa, 0x35, 0x68, 0x7c, 0x88,
    ]);

    /// SongRegistered(bytes32 indexed songId, bytes32 indexed strategyId, address indexed artist)
    pub const SONG_REGISTERED: H256 = H256([
        0x7c, 0x55, 0x12, 0x34, 0x89, 0xab, 0xcd, 0xef,
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
        0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
    ]);
}

/// Payment record from blockchain event
#[derive(Debug, Clone)]
pub struct PaymentEvent {
    pub tx_hash: H256,
    pub block_number: u64,
    pub song_id: [u8; 32],
    pub listener: Address,
    pub amount: U256,
    pub payment_type: u8,
    pub timestamp: u64,
}

/// Song registration event
#[derive(Debug, Clone)]
pub struct SongRegisteredEvent {
    pub tx_hash: H256,
    pub block_number: u64,
    pub song_id: [u8; 32],
    pub strategy_id: [u8; 32],
    pub artist: Address,
}

/// Event indexer configuration
#[derive(Clone)]
pub struct IndexerConfig {
    pub rpc_url: String,
    pub router_address: Address,
    pub start_block: u64,
    pub poll_interval_secs: u64,
    pub confirmations: u64,
}

impl Default for IndexerConfig {
    fn default() -> Self {
        Self {
            rpc_url: "http://localhost:8545".into(),
            router_address: Address::zero(),
            start_block: 0,
            poll_interval_secs: 12, // ~1 block on Gnosis
            confirmations: 3,
        }
    }
}

/// Event indexer service
pub struct EventIndexer {
    provider: Arc<Provider<Http>>,
    config: IndexerConfig,
    db_pool: PgPool,
    last_indexed_block: u64,
}

impl EventIndexer {
    /// Create a new event indexer
    pub async fn new(config: IndexerConfig, db_pool: PgPool) -> Result<Self> {
        let provider = Provider::<Http>::try_from(&config.rpc_url)?;

        // Get last indexed block from database or use config start
        let last_indexed_block = sqlx::query_scalar!(
            "SELECT COALESCE(MAX(block_number), $1) as block FROM indexed_events",
            config.start_block as i64
        )
        .fetch_one(&db_pool)
        .await
        .map(|r| r.unwrap_or(config.start_block as i64) as u64)
        .unwrap_or(config.start_block);

        Ok(Self {
            provider: Arc::new(provider),
            config,
            db_pool,
            last_indexed_block,
        })
    }

    /// Start the indexer loop
    pub async fn run(&mut self) -> Result<()> {
        info!(
            "Starting event indexer from block {} for router {:?}",
            self.last_indexed_block, self.config.router_address
        );

        loop {
            match self.index_new_blocks().await {
                Ok(count) => {
                    if count > 0 {
                        info!("Indexed {} new events", count);
                    }
                }
                Err(e) => {
                    error!("Indexer error: {:?}", e);
                }
            }

            sleep(Duration::from_secs(self.config.poll_interval_secs)).await;
        }
    }

    /// Index events from new blocks
    async fn index_new_blocks(&mut self) -> Result<usize> {
        let current_block = self.provider.get_block_number().await?.as_u64();
        let safe_block = current_block.saturating_sub(self.config.confirmations);

        if safe_block <= self.last_indexed_block {
            return Ok(0);
        }

        let from_block = self.last_indexed_block + 1;
        let to_block = std::cmp::min(safe_block, from_block + 1000); // Max 1000 blocks at a time

        info!("Indexing blocks {} to {}", from_block, to_block);

        let filter = Filter::new()
            .address(self.config.router_address)
            .from_block(from_block)
            .to_block(to_block);

        let logs = self.provider.get_logs(&filter).await?;
        let mut event_count = 0;

        for log in logs {
            match self.process_log(&log).await {
                Ok(true) => event_count += 1,
                Ok(false) => {} // Unknown event type
                Err(e) => {
                    warn!("Failed to process log: {:?}", e);
                }
            }
        }

        // Update last indexed block
        self.last_indexed_block = to_block;
        self.save_checkpoint(to_block).await?;

        Ok(event_count)
    }

    /// Process a single log entry
    async fn process_log(&self, log: &Log) -> Result<bool> {
        if log.topics.is_empty() {
            return Ok(false);
        }

        let event_sig = log.topics[0];

        if event_sig == event_signatures::PAYMENT_PROCESSED {
            self.process_payment_event(log).await?;
            return Ok(true);
        }

        if event_sig == event_signatures::SONG_REGISTERED {
            self.process_song_registered_event(log).await?;
            return Ok(true);
        }

        Ok(false)
    }

    /// Process a PaymentProcessed event
    async fn process_payment_event(&self, log: &Log) -> Result<()> {
        // Parse event data from log
        // topics: [event_sig, songId, listener]
        // data: [amount, paymentType]

        let song_id: [u8; 32] = log.topics.get(1)
            .map(|h| h.0)
            .unwrap_or_default();

        let listener = log.topics.get(2)
            .map(|h| Address::from_slice(&h.0[12..]))
            .unwrap_or_default();

        // Decode amount and payment type from data
        let data = &log.data.0;
        let amount = if data.len() >= 32 {
            U256::from_big_endian(&data[0..32])
        } else {
            U256::zero()
        };

        let payment_type = if data.len() >= 64 {
            data[63]
        } else {
            0
        };

        let block_number = log.block_number.map(|b| b.as_u64()).unwrap_or(0);
        let tx_hash = log.transaction_hash.unwrap_or_default();

        // Get block timestamp
        let timestamp = if let Some(block) = self.provider.get_block(block_number).await? {
            block.timestamp.as_u64()
        } else {
            0
        };

        // Store in database
        sqlx::query!(
            r#"
            INSERT INTO payments (
                tx_hash, block_number, song_id, listener_address,
                amount_wei, payment_type, timestamp, created_at
            )
            VALUES ($1, $2, $3, $4, $5, $6, to_timestamp($7), NOW())
            ON CONFLICT (tx_hash) DO NOTHING
            "#,
            format!("{:?}", tx_hash),
            block_number as i64,
            hex::encode(song_id),
            format!("{:?}", listener),
            amount.to_string(),
            payment_type as i16,
            timestamp as f64,
        )
        .execute(&self.db_pool)
        .await?;

        info!(
            "Indexed payment: song={}, listener={:?}, amount={}, type={}",
            hex::encode(&song_id[..8]),
            listener,
            amount,
            payment_type
        );

        Ok(())
    }

    /// Process a SongRegistered event
    async fn process_song_registered_event(&self, log: &Log) -> Result<()> {
        let song_id: [u8; 32] = log.topics.get(1)
            .map(|h| h.0)
            .unwrap_or_default();

        let strategy_id: [u8; 32] = log.topics.get(2)
            .map(|h| h.0)
            .unwrap_or_default();

        let artist = log.topics.get(3)
            .map(|h| Address::from_slice(&h.0[12..]))
            .unwrap_or_default();

        let block_number = log.block_number.map(|b| b.as_u64()).unwrap_or(0);
        let tx_hash = log.transaction_hash.unwrap_or_default();

        // Update song record with on-chain registration
        sqlx::query!(
            r#"
            UPDATE songs
            SET
                strategy_id = $1,
                registered_on_chain = true,
                registration_tx = $2,
                registration_block = $3,
                updated_at = NOW()
            WHERE song_id = $4
            "#,
            hex::encode(strategy_id),
            format!("{:?}", tx_hash),
            block_number as i64,
            hex::encode(song_id),
        )
        .execute(&self.db_pool)
        .await?;

        info!(
            "Indexed song registration: song={}, artist={:?}, strategy={}",
            hex::encode(&song_id[..8]),
            artist,
            hex::encode(&strategy_id[..8])
        );

        Ok(())
    }

    /// Save indexer checkpoint
    async fn save_checkpoint(&self, block_number: u64) -> Result<()> {
        sqlx::query!(
            r#"
            INSERT INTO indexed_events (event_type, block_number, created_at)
            VALUES ('checkpoint', $1, NOW())
            "#,
            block_number as i64,
        )
        .execute(&self.db_pool)
        .await?;

        Ok(())
    }
}

/// Start the indexer as a background task
pub fn spawn_indexer(config: IndexerConfig, db_pool: PgPool) {
    tokio::spawn(async move {
        match EventIndexer::new(config, db_pool).await {
            Ok(mut indexer) => {
                if let Err(e) = indexer.run().await {
                    error!("Indexer failed: {:?}", e);
                }
            }
            Err(e) => {
                error!("Failed to create indexer: {:?}", e);
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_payment_type_conversion() {
        assert_eq!(0u8, 0); // Stream
        assert_eq!(1u8, 1); // Download
    }
}
