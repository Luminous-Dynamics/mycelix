//! Typed Marketplace coordinator client over an injected transport.

pub mod contract;

use async_trait::async_trait;
use marketplace_domain::{
    ActionHash, AgentPubKey, ApplyArbitrationTransactionConflictInput,
    ApproveTransactionConflictInput, ArbitrationResultOutput, ArbitrationVoteOutput,
    ArbitrationVotesResponse, CreateListingInput, CreateTransactionInput, DerivedReputation,
    DisputeOutput, DisputeResolution, DisputesResponse, FileDisputeInput,
    FinalizeBilateralTransactionConflictInput, ListingOutput, ListingsResponse, MarkShippedInput,
    OpenDisputeInput, OpenDisputeOutput, ReputationEventOutput, ReputationEventsResponse,
    SubmitArbitrationVoteInput, TransactionConflictApprovalOutput,
    TransactionConflictResolutionOutput, TransactionOutput, TransactionResolution,
    TransactionResolutionsResponse, TransactionSettlementResult, TransactionsResponse,
};
use serde::{Serialize, de::DeserializeOwned};

#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    #[error("live Marketplace transport is unavailable: {0}")]
    Unavailable(String),
    #[error("failed to encode zome input: {0}")]
    Encode(String),
    #[error("failed to decode zome output: {0}")]
    Decode(String),
    #[error("zome call {zome}.{function} failed: {message}")]
    Call {
        zome: String,
        function: String,
        message: String,
    },
}

#[async_trait(?Send)]
pub trait ZomeTransport: Clone + 'static {
    async fn call_zome(
        &self,
        role: &str,
        zome: &str,
        function: &str,
        payload: Vec<u8>,
    ) -> Result<Vec<u8>, ClientError>;
}

#[derive(Clone)]
pub struct MarketplaceClient<T> {
    transport: T,
}

impl<T> MarketplaceClient<T>
where
    T: ZomeTransport,
{
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    async fn call<I, O>(&self, zome: &str, function: &str, input: &I) -> Result<O, ClientError>
    where
        I: Serialize + Sync,
        O: DeserializeOwned,
    {
        let payload = rmp_serde::to_vec_named(input)
            .map_err(|error| ClientError::Encode(error.to_string()))?;
        let output = self
            .transport
            .call_zome(contract::ROLE, zome, function, payload)
            .await?;
        rmp_serde::from_slice(&output).map_err(|error| ClientError::Decode(error.to_string()))
    }

    pub async fn get_all_listings(&self) -> Result<ListingsResponse, ClientError> {
        self.call(
            contract::listings::ZOME,
            contract::listings::GET_ALL_LISTINGS,
            &(),
        )
        .await
    }

    pub async fn get_listing(
        &self,
        listing_hash: &ActionHash,
    ) -> Result<Option<ListingOutput>, ClientError> {
        self.call(
            contract::listings::ZOME,
            contract::listings::GET_LISTING,
            listing_hash,
        )
        .await
    }

    pub async fn create_listing(
        &self,
        input: &CreateListingInput,
    ) -> Result<ListingOutput, ClientError> {
        self.call(
            contract::listings::ZOME,
            contract::listings::CREATE_LISTING,
            input,
        )
        .await
    }

    pub async fn create_transaction(
        &self,
        input: &CreateTransactionInput,
    ) -> Result<TransactionOutput, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::CREATE_TRANSACTION,
            input,
        )
        .await
    }

    pub async fn get_transaction(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<Option<TransactionOutput>, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::GET_TRANSACTION,
            transaction_hash,
        )
        .await
    }

    pub async fn get_transaction_resolution(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<Option<TransactionResolution>, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::GET_TRANSACTION_RESOLUTION,
            transaction_hash,
        )
        .await
    }

    pub async fn approve_transaction_conflict(
        &self,
        input: &ApproveTransactionConflictInput,
    ) -> Result<TransactionConflictApprovalOutput, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::APPROVE_TRANSACTION_CONFLICT,
            input,
        )
        .await
    }

    pub async fn finalize_bilateral_transaction_conflict(
        &self,
        input: &FinalizeBilateralTransactionConflictInput,
    ) -> Result<TransactionResolution, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::FINALIZE_BILATERAL_TRANSACTION_CONFLICT,
            input,
        )
        .await
    }

    pub async fn apply_arbitration_transaction_conflict(
        &self,
        input: &ApplyArbitrationTransactionConflictInput,
    ) -> Result<TransactionResolution, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::APPLY_ARBITRATION_TRANSACTION_CONFLICT,
            input,
        )
        .await
    }

    pub async fn get_transaction_conflict_approvals(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<Vec<TransactionConflictApprovalOutput>, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::GET_TRANSACTION_CONFLICT_APPROVALS,
            transaction_hash,
        )
        .await
    }

    pub async fn get_transaction_conflict_resolutions(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<Vec<TransactionConflictResolutionOutput>, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::GET_TRANSACTION_CONFLICT_RESOLUTIONS,
            transaction_hash,
        )
        .await
    }

    pub async fn get_my_transactions(&self) -> Result<TransactionsResponse, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::GET_MY_TRANSACTIONS,
            &(),
        )
        .await
    }
    pub async fn get_my_transaction_resolutions(
        &self,
    ) -> Result<TransactionResolutionsResponse, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::GET_MY_TRANSACTION_RESOLUTIONS,
            &(),
        )
        .await
    }

    pub async fn confirm_transaction(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<TransactionOutput, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::CONFIRM_TRANSACTION,
            transaction_hash,
        )
        .await
    }

    pub async fn mark_shipped(
        &self,
        input: &MarkShippedInput,
    ) -> Result<TransactionOutput, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::MARK_SHIPPED,
            input,
        )
        .await
    }

    pub async fn confirm_delivery(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<TransactionOutput, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::CONFIRM_DELIVERY,
            transaction_hash,
        )
        .await
    }

    pub async fn cancel_transaction(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<TransactionOutput, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::CANCEL_TRANSACTION,
            transaction_hash,
        )
        .await
    }

    pub async fn settle_transaction(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<TransactionSettlementResult, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::SETTLE_TRANSACTION,
            transaction_hash,
        )
        .await
    }

    pub async fn get_transaction_settlement_status(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<TransactionSettlementResult, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::GET_TRANSACTION_SETTLEMENT_STATUS,
            transaction_hash,
        )
        .await
    }

    pub async fn record_fulfillment_reputation(
        &self,
        transaction_hash: &ActionHash,
    ) -> Result<ReputationEventOutput, ClientError> {
        self.call(
            contract::reputation::ZOME,
            contract::reputation::RECORD_FULFILLMENT_REPUTATION,
            transaction_hash,
        )
        .await
    }

    pub async fn project_arbitration_reputation(
        &self,
        result_hash: &ActionHash,
    ) -> Result<ReputationEventsResponse, ClientError> {
        self.call(
            contract::reputation::ZOME,
            contract::reputation::PROJECT_ARBITRATION_REPUTATION,
            result_hash,
        )
        .await
    }

    pub async fn get_agent_reputation_events(
        &self,
        agent: &AgentPubKey,
    ) -> Result<ReputationEventsResponse, ClientError> {
        self.call(
            contract::reputation::ZOME,
            contract::reputation::GET_AGENT_REPUTATION_EVENTS,
            agent,
        )
        .await
    }

    pub async fn get_derived_reputation(
        &self,
        agent: &AgentPubKey,
    ) -> Result<DerivedReputation, ClientError> {
        self.call(
            contract::reputation::ZOME,
            contract::reputation::GET_DERIVED_REPUTATION,
            agent,
        )
        .await
    }

    pub async fn open_dispute(
        &self,
        input: &OpenDisputeInput,
    ) -> Result<OpenDisputeOutput, ClientError> {
        self.call(
            contract::transactions::ZOME,
            contract::transactions::OPEN_DISPUTE,
            input,
        )
        .await
    }

    pub async fn file_transaction_conflict_dispute(
        &self,
        input: &FileDisputeInput,
    ) -> Result<DisputeOutput, ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::FILE_TRANSACTION_CONFLICT_DISPUTE,
            input,
        )
        .await
    }

    pub async fn register_as_arbitrator(&self) -> Result<(), ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::REGISTER_AS_ARBITRATOR,
            &(),
        )
        .await
    }

    pub async fn get_arbitration_opportunities(&self) -> Result<DisputesResponse, ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::GET_ARBITRATION_OPPORTUNITIES,
            &(),
        )
        .await
    }

    pub async fn get_dispute(
        &self,
        dispute_hash: &ActionHash,
    ) -> Result<Option<DisputeOutput>, ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::GET_DISPUTE,
            dispute_hash,
        )
        .await
    }

    pub async fn get_dispute_resolution(
        &self,
        dispute_hash: &ActionHash,
    ) -> Result<Option<DisputeResolution>, ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::GET_DISPUTE_RESOLUTION,
            dispute_hash,
        )
        .await
    }

    pub async fn submit_arbitration_vote(
        &self,
        input: &SubmitArbitrationVoteInput,
    ) -> Result<ArbitrationVoteOutput, ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::SUBMIT_ARBITRATION_VOTE,
            input,
        )
        .await
    }

    pub async fn get_arbitration_votes(
        &self,
        dispute_hash: &ActionHash,
    ) -> Result<ArbitrationVotesResponse, ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::GET_ARBITRATION_VOTES,
            dispute_hash,
        )
        .await
    }

    pub async fn finalize_arbitration(
        &self,
        dispute_hash: &ActionHash,
    ) -> Result<ArbitrationResultOutput, ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::FINALIZE_ARBITRATION,
            dispute_hash,
        )
        .await
    }

    pub async fn get_arbitration_result(
        &self,
        dispute_hash: &ActionHash,
    ) -> Result<Option<ArbitrationResultOutput>, ClientError> {
        self.call(
            contract::arbitration::ZOME,
            contract::arbitration::GET_ARBITRATION_RESULT,
            dispute_hash,
        )
        .await
    }
}

#[derive(Clone, Default)]
pub struct UnavailableTransport;

#[async_trait(?Send)]
impl ZomeTransport for UnavailableTransport {
    async fn call_zome(
        &self,
        _role: &str,
        _zome: &str,
        _function: &str,
        _payload: Vec<u8>,
    ) -> Result<Vec<u8>, ClientError> {
        Err(ClientError::Unavailable(
            "configure an authenticated signed Holochain transport".into(),
        ))
    }
}

#[cfg(feature = "dev-fixtures")]
mod fixtures;
#[cfg(feature = "dev-fixtures")]
pub use fixtures::{FixtureTransport, fixture_agent, fixture_listings};

#[cfg(feature = "js-bridge")]
mod js_bridge;
#[cfg(feature = "js-bridge")]
pub use js_bridge::{BridgeConnectionInfo, JsBridgeTransport};
