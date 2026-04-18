// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Blockchain Service - Smart contract interactions
//!
//! Handles all interactions with the EconomicStrategyRouter
//! and payment processing on Gnosis Chain.

use anyhow::Result;
use ethers::prelude::*;
use std::sync::Arc;

// Generate type-safe contract bindings for the PaymentRouter
abigen!(
    PaymentRouter,
    r#"[
        function songStrategy(bytes32 songId) external view returns (address)
        function processPayment(bytes32 songId, uint256 amount, uint8 paymentType) external
        function previewSplits(bytes32 songId, uint256 amount) external view returns (tuple(address recipient, uint256 basisPoints, string role)[])
        function flowToken() external view returns (address)
        event PaymentProcessed(bytes32 indexed songId, address indexed payer, uint256 amount, uint8 paymentType)
    ]"#
);

/// Blockchain service for contract interactions
pub struct BlockchainService {
    provider: Arc<Provider<Http>>,
    router_address: Address,
    signer: Option<Arc<SignerMiddleware<Provider<Http>, LocalWallet>>>,
}

impl BlockchainService {
    pub fn new(rpc_url: &str, router_address: &str) -> Result<Self> {
        let provider = Provider::<Http>::try_from(rpc_url)?;
        let router_address = router_address.parse()?;

        Ok(Self {
            provider: Arc::new(provider),
            router_address,
            signer: None,
        })
    }

    /// Create a new service with a signer for transactions
    pub fn with_signer(rpc_url: &str, router_address: &str, private_key: &str) -> Result<Self> {
        let provider = Provider::<Http>::try_from(rpc_url)?;
        let router_address: Address = router_address.parse()?;

        // Parse the private key and create a wallet
        let wallet: LocalWallet = private_key.parse()?;
        let chain_id = 100u64; // Gnosis Chain

        let signer = SignerMiddleware::new(
            provider.clone(),
            wallet.with_chain_id(chain_id),
        );

        Ok(Self {
            provider: Arc::new(provider),
            router_address,
            signer: Some(Arc::new(signer)),
        })
    }

    /// Get current block number
    pub async fn get_block_number(&self) -> Result<u64> {
        Ok(self.provider.get_block_number().await?.as_u64())
    }

    /// Verify a signature
    pub fn verify_signature(
        &self,
        message: &[u8],
        signature: &[u8],
        expected_signer: Address,
    ) -> Result<bool> {
        let signature = Signature::try_from(signature)?;
        let recovered = signature.recover(message)?;
        Ok(recovered == expected_signer)
    }

    /// Get strategy address for a song
    pub async fn get_song_strategy(&self, song_id: [u8; 32]) -> Result<Option<Address>> {
        let router = PaymentRouter::new(self.router_address, self.provider.clone());

        // Call router.songStrategy(songId)
        let strategy_address = router.song_strategy(song_id).call().await?;

        // Return None if the strategy is the zero address (not registered)
        if strategy_address == Address::zero() {
            Ok(None)
        } else {
            Ok(Some(strategy_address))
        }
    }

    /// Process a payment through the router
    pub async fn process_payment(
        &self,
        song_id: [u8; 32],
        amount: U256,
        payment_type: u8,
    ) -> Result<H256> {
        let signer = self.signer.as_ref()
            .ok_or_else(|| anyhow::anyhow!("Signer required for payment processing"))?;

        let router = PaymentRouter::new(self.router_address, signer.clone());

        // Call router.processPayment(songId, amount, paymentType)
        let tx = router.process_payment(song_id, amount, payment_type);

        // Send the transaction and wait for confirmation
        let pending_tx = tx.send().await?;
        let receipt = pending_tx
            .await?
            .ok_or_else(|| anyhow::anyhow!("Transaction failed - no receipt"))?;

        Ok(receipt.transaction_hash)
    }

    /// Preview how a payment would be split
    pub async fn preview_splits(
        &self,
        song_id: [u8; 32],
        amount: U256,
    ) -> Result<Vec<PaymentSplit>> {
        let router = PaymentRouter::new(self.router_address, self.provider.clone());

        let splits = router.preview_splits(song_id, amount).call().await?;

        Ok(splits
            .into_iter()
            .map(|s| PaymentSplit {
                recipient: s.recipient,
                basis_points: s.basis_points.as_u32(),
                role: s.role,
            })
            .collect())
    }

    /// Get the FLOW token address from the router
    pub async fn get_flow_token_address(&self) -> Result<Address> {
        let router = PaymentRouter::new(self.router_address, self.provider.clone());
        Ok(router.flow_token().call().await?)
    }

    /// Get the router address
    pub fn router_address(&self) -> Address {
        self.router_address
    }
}

/// Represents a payment split recipient
#[derive(Debug, Clone)]
pub struct PaymentSplit {
    pub recipient: Address,
    pub basis_points: u32,
    pub role: String,
}

/// Payment types matching the contract enum
#[derive(Debug, Clone, Copy)]
pub enum PaymentType {
    Stream = 0,
    Download = 1,
    Tip = 2,
    Patronage = 3,
    NftAccess = 4,
}

impl From<PaymentType> for u8 {
    fn from(pt: PaymentType) -> u8 {
        pt as u8
    }
}

impl TryFrom<&str> for PaymentType {
    type Error = anyhow::Error;

    fn try_from(s: &str) -> Result<Self> {
        match s.to_lowercase().as_str() {
            "stream" => Ok(PaymentType::Stream),
            "download" => Ok(PaymentType::Download),
            "tip" => Ok(PaymentType::Tip),
            "patronage" => Ok(PaymentType::Patronage),
            "nft_access" | "nftaccess" => Ok(PaymentType::NftAccess),
            _ => Err(anyhow::anyhow!("Unknown payment type: {}", s)),
        }
    }
}
