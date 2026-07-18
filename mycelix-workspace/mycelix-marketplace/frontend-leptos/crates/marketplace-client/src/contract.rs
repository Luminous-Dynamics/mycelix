//! Exact coordinator names. Pages must call typed methods instead of these strings.

pub const ROLE: &str = "marketplace";

pub mod listings {
    pub const ZOME: &str = "listings";
    pub const CREATE_LISTING: &str = "create_listing";
    pub const GET_LISTING: &str = "get_listing";
    pub const GET_ALL_LISTINGS: &str = "get_all_listings";
    pub const GET_MY_LISTINGS: &str = "get_my_listings";
    pub const SEARCH_LISTINGS: &str = "search_listings";
    pub const UPDATE_LISTING: &str = "update_listing";
    pub const DELETE_LISTING: &str = "delete_listing";
}

pub mod transactions {
    pub const ZOME: &str = "transactions";
    pub const CREATE_TRANSACTION: &str = "create_transaction";
    pub const GET_TRANSACTION: &str = "get_transaction";
    pub const GET_TRANSACTION_RESOLUTION: &str = "get_transaction_resolution";
    pub const APPROVE_TRANSACTION_CONFLICT: &str = "approve_transaction_conflict";
    pub const FINALIZE_BILATERAL_TRANSACTION_CONFLICT: &str = "finalize_bilateral_transaction_conflict";
    pub const APPLY_ARBITRATION_TRANSACTION_CONFLICT: &str = "apply_arbitration_transaction_conflict";
    pub const GET_TRANSACTION_CONFLICT_APPROVALS: &str = "get_transaction_conflict_approvals";
    pub const GET_TRANSACTION_CONFLICT_RESOLUTIONS: &str = "get_transaction_conflict_resolutions";
    pub const GET_MY_TRANSACTIONS: &str = "get_my_transactions";
    pub const GET_MY_TRANSACTION_RESOLUTIONS: &str = "get_my_transaction_resolutions";
    pub const CONFIRM_TRANSACTION: &str = "confirm_transaction";
    pub const MARK_SHIPPED: &str = "mark_shipped";
    pub const CONFIRM_DELIVERY: &str = "confirm_delivery";
    pub const COMPLETE_TRANSACTION: &str = "complete_transaction";
    pub const SETTLE_TRANSACTION: &str = "settle_transaction";
    pub const GET_TRANSACTION_SETTLEMENT_STATUS: &str = "get_transaction_settlement_status";
    pub const OPEN_DISPUTE: &str = "open_dispute";
    pub const DISPUTE_TRANSACTION: &str = "dispute_transaction";
    pub const CANCEL_TRANSACTION: &str = "cancel_transaction";
    pub const GET_LISTING_TRANSACTIONS: &str = "get_listing_transactions";
}


pub mod arbitration {
    pub const ZOME: &str = "arbitration";
    pub const FILE_DISPUTE: &str = "file_dispute";
    pub const FILE_TRANSACTION_CONFLICT_DISPUTE: &str = "file_transaction_conflict_dispute";
    pub const REGISTER_AS_ARBITRATOR: &str = "register_as_arbitrator";
    pub const SUBMIT_ARBITRATION_VOTE: &str = "submit_arbitration_vote";
    pub const FINALIZE_ARBITRATION: &str = "finalize_arbitration";
    pub const GET_ARBITRATION_OPPORTUNITIES: &str = "get_arbitration_opportunities";
    pub const GET_DISPUTE: &str = "get_dispute";
    pub const GET_DISPUTE_RESOLUTION: &str = "get_dispute_resolution";
    pub const GET_ARBITRATION_VOTES: &str = "get_arbitration_votes";
    pub const GET_ARBITRATION_RESULT: &str = "get_arbitration_result";
}


pub mod reputation {
    pub const ZOME: &str = "reputation";
    pub const RECORD_FULFILLMENT_REPUTATION: &str = "record_fulfillment_reputation";
    pub const PROJECT_ARBITRATION_REPUTATION: &str = "project_arbitration_reputation";
    pub const GET_AGENT_REPUTATION_EVENTS: &str = "get_agent_reputation_events";
    pub const GET_DERIVED_REPUTATION: &str = "get_derived_reputation";
}
