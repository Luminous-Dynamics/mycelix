// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Flow Integrity Zome
//! Water allocation, H2O credits, and water economics
//!
//! The FLOW pillar manages water sources, share allocations,
//! credit balances, and transaction records for community water systems.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

// ============================================================================
// WATER SOURCE
// ============================================================================

/// Type of water source
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum WaterSourceType {
    Municipal,
    Well,
    Spring,
    Rainwater,
    Aquifer,
    River,
    Lake,
    Recycled,
    Desalinated,
}

/// Operational status of a water source
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SourceStatus {
    Active,
    Seasonal,
    Depleted,
    Contaminated,
    UnderMaintenance,
}

/// A registered water source in the community system
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WaterSource {
    /// Unique identifier for this source
    pub id: String,
    /// Human-readable name
    pub name: String,
    /// Type of water source
    pub source_type: WaterSourceType,
    /// Maximum capacity in liters
    pub max_capacity_liters: u64,
    /// Natural recharge rate in liters per day
    pub recharge_rate_liters_per_day: u64,
    /// GPS latitude
    pub location_lat: f64,
    /// GPS longitude
    pub location_lon: f64,
    /// Agent responsible for this source
    pub steward: AgentPubKey,
    /// Current operational status
    pub status: SourceStatus,
}

// ============================================================================
// WATER SHARES
// ============================================================================

/// How water is allocated from a source
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AllocationType {
    /// Fixed volume per period
    Fixed,
    /// Proportional to total available
    Proportional,
    /// Priority-based during scarcity
    Priority,
    /// Emergency allocation
    Emergency,
}

/// Classification of water use
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum WaterClassification {
    Potable,
    Cooking,
    Hygiene,
    Irrigation,
    Industrial,
    Recreation,
    Greywater,
}

/// A share of water allocated from a source to a holder
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WaterShare {
    /// Hash of the WaterSource this share draws from
    pub source_hash: ActionHash,
    /// Agent holding this share
    pub holder: AgentPubKey,
    /// How this allocation works
    pub allocation_type: AllocationType,
    /// Volume allocated per period in liters
    pub volume_per_period_liters: u64,
    /// Period length in days
    pub period_days: u32,
    /// Priority level (0 = highest priority)
    pub priority: u8,
    /// What this water is used for
    pub usage_category: WaterClassification,
}

// ============================================================================
// H2O CREDITS
// ============================================================================

/// H2O credit balance for an agent
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct H2OCredit {
    /// Agent holding these credits
    pub holder: AgentPubKey,
    /// Current balance in liters (can go negative for overdraft)
    pub balance_liters: i64,
    /// Total credits ever earned
    pub total_earned: u64,
    /// Total credits ever spent
    pub total_spent: u64,
}

// ============================================================================
// WATER TRANSACTIONS
// ============================================================================

/// Type of water credit transaction
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TransactionType {
    /// Regular allocation from source
    Allocation,
    /// Peer-to-peer transfer
    Transfer,
    /// Purchase of credits
    Purchase,
    /// Donation of credits
    Donation,
    /// Emergency allocation
    Emergency,
}

/// A record of water credit movement between agents
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct WaterTransaction {
    /// Sending agent
    pub from_agent: AgentPubKey,
    /// Receiving agent
    pub to_agent: AgentPubKey,
    /// Volume in liters
    pub liters: u64,
    /// Type of transaction
    pub credit_type: TransactionType,
    /// When this transaction occurred
    pub timestamp: Timestamp,
    /// Optional link to water source
    pub source_hash: Option<ActionHash>,
}

// ============================================================================
// USAGE RECORD
// ============================================================================

/// Record of actual water usage against an allocation
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct UsageRecord {
    /// Agent who used the water
    pub agent: AgentPubKey,
    /// Source from which water was drawn
    pub source_hash: ActionHash,
    /// Liters consumed
    pub liters_used: u64,
    /// What the water was used for
    pub usage_category: WaterClassification,
    /// When usage was recorded
    pub recorded_at: Timestamp,
    /// Optional meter reading or sensor data reference
    pub meter_reference: Option<String>,
}

// ============================================================================
// ENTRY & LINK TYPE REGISTRATION
// ============================================================================

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    WaterSource(WaterSource),
    WaterShare(WaterShare),
    H2OCredit(H2OCredit),
    WaterTransaction(WaterTransaction),
    UsageRecord(UsageRecord),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor to all water sources
    AllSources,
    /// Source type anchor to sources of that type
    SourceTypeToSource,
    /// Steward agent to their sources
    StewardToSource,
    /// Source to its allocated shares
    SourceToShare,
    /// Holder agent to their shares
    HolderToShare,
    /// Agent to their credit balance
    AgentToCredit,
    /// Agent to their transactions (sent)
    AgentToTransactionSent,
    /// Agent to their transactions (received)
    AgentToTransactionReceived,
    /// Source to usage records
    SourceToUsage,
    /// Agent to their usage records
    AgentToUsage,
}

// ============================================================================
// VALIDATION
// ============================================================================

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::WaterSource(source) => validate_create_water_source(action, source),
                EntryTypes::WaterShare(share) => validate_create_water_share(action, share),
                EntryTypes::H2OCredit(credit) => validate_create_h2o_credit(action, credit),
                EntryTypes::WaterTransaction(tx) => validate_create_water_transaction(action, tx),
                EntryTypes::UsageRecord(usage) => validate_create_usage_record(action, usage),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::WaterSource(source) => {
                    validate_update_water_source(action, source, original_action_hash)
                }
                EntryTypes::H2OCredit(credit) => {
                    validate_update_h2o_credit(action, credit, original_action_hash)
                }
                _ => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::AllSources => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SourceTypeToSource => Ok(ValidateCallbackResult::Valid),
            LinkTypes::StewardToSource => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SourceToShare => Ok(ValidateCallbackResult::Valid),
            LinkTypes::HolderToShare => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToCredit => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToTransactionSent => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToTransactionReceived => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SourceToUsage => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToUsage => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_water_source(
    action: Create,
    source: WaterSource,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the source to its committer -- register_source already derives
    // `steward` from agent_info() coordinator-side with zero user input, so
    // this never rejects a legitimate registration; it's the real
    // DHT-level enforcement a modified coordinator could otherwise bypass
    // (P0 author-binding gap).
    if source.steward != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Water source steward must be the committing agent (forgery)".to_string(),
        ));
    }

    if source.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Water source ID cannot be empty".into(),
        ));
    }
    if source.name.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Water source name cannot be empty".into(),
        ));
    }
    if source.max_capacity_liters == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Max capacity must be greater than zero".into(),
        ));
    }
    if source.location_lat < -90.0 || source.location_lat > 90.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude must be between -90 and 90".into(),
        ));
    }
    if source.location_lon < -180.0 || source.location_lon > 180.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Longitude must be between -180 and 180".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_water_source(
    action: Update,
    source: WaterSource,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original_source: WaterSource = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original water source not found".into()
        )))?;

    // Bind the update to the PRE-update steward -- update_source_status
    // already checks this coordinator-side, but that trusts the
    // coordinator; this is the real DHT-level enforcement a modified
    // coordinator can't bypass (P0 author-binding gap).
    if action.author != original_source.steward {
        return Ok(ValidateCallbackResult::Invalid(
            "Water source update must be committed by its steward".to_string(),
        ));
    }

    if source.id != original_source.id {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot change water source ID".into(),
        ));
    }
    if source.steward != original_source.steward {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot change water source steward via update; use transfer".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_water_share(
    action: Create,
    share: WaterShare,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the allocation to the source's steward -- allocate_shares already
    // checks `source.steward == caller` coordinator-side, but that trusts
    // the coordinator; this is the real DHT-level enforcement a modified
    // coordinator can't bypass (P0 author-binding gap). holder is
    // deliberately NOT bound to the committer -- allocation is legitimately
    // steward-to-third-party.
    let source_record = must_get_valid_record(share.source_hash.clone())?;
    let source: WaterSource = source_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Water source not found".to_string()
        )))?;
    if action.author != source.steward {
        return Ok(ValidateCallbackResult::Invalid(
            "Water share must be allocated by the source's steward".to_string(),
        ));
    }

    if share.volume_per_period_liters == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Share volume must be greater than zero".into(),
        ));
    }
    if share.period_days == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Period days must be greater than zero".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_h2o_credit(
    action: Create,
    credit: H2OCredit,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the credit balance to its committer -- get_my_balance only ever
    // initializes a balance for the calling agent itself, with zero user
    // input on `holder` (P0 author-binding gap).
    if credit.holder != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "H2O credit holder must be the committing agent (forgery)".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_h2o_credit(
    action: Update,
    credit: H2OCredit,
    _original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the update to its committer -- transfer_credits/record_usage
    // only ever debit/credit the calling agent's OWN balance; holder never
    // changes across updates in this design, so checking the new entry
    // directly (no must_get needed) is sufficient.
    if credit.holder != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "H2O credit update must be committed by its holder (forgery)".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_water_transaction(
    action: Create,
    tx: WaterTransaction,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the transaction to its sender -- transfer_credits already
    // derives `from_agent` from agent_info() coordinator-side with zero
    // user input. to_agent is deliberately NOT bound -- it legitimately
    // names the recipient, a third party.
    if tx.from_agent != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Transaction from_agent must be the committing agent (forgery)".to_string(),
        ));
    }

    if tx.liters == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Transaction volume must be greater than zero".into(),
        ));
    }
    if tx.from_agent == tx.to_agent {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot transfer credits to self".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_usage_record(
    action: Create,
    usage: UsageRecord,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the record to its committer -- record_usage already derives
    // `agent` from agent_info() coordinator-side with zero user input.
    if usage.agent != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Usage record agent must be the committing agent (forgery)".to_string(),
        ));
    }

    if usage.liters_used == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Usage liters must be greater than zero".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_action() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_source(steward: AgentPubKey) -> WaterSource {
        WaterSource {
            id: "source-1".to_string(),
            name: "Community Well".to_string(),
            source_type: WaterSourceType::Well,
            max_capacity_liters: 10000,
            recharge_rate_liters_per_day: 100,
            location_lat: 10.0,
            location_lon: 20.0,
            steward,
            status: SourceStatus::Active,
        }
    }

    #[test]
    fn test_create_water_source_valid() {
        let source = valid_source(test_action().author);
        let result = validate_create_water_source(test_action(), source).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_water_source_steward_forgery_rejected() {
        let mut forged_action = test_action();
        forged_action.author = other_agent();
        let source = valid_source(test_action().author);
        let result = validate_create_water_source(forged_action, source).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_h2o_credit_valid() {
        let credit = H2OCredit {
            holder: test_action().author,
            balance_liters: 0,
            total_earned: 0,
            total_spent: 0,
        };
        let result = validate_create_h2o_credit(test_action(), credit).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_h2o_credit_holder_forgery_rejected() {
        let credit = H2OCredit {
            holder: other_agent(),
            balance_liters: 0,
            total_earned: 0,
            total_spent: 0,
        };
        let result = validate_create_h2o_credit(test_action(), credit).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_update_h2o_credit_holder_forgery_rejected() {
        let action = Update {
            author: test_action().author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![0u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        };
        let credit = H2OCredit {
            holder: other_agent(),
            balance_liters: 100,
            total_earned: 100,
            total_spent: 0,
        };
        let result =
            validate_update_h2o_credit(action, credit, ActionHash::from_raw_36(vec![0u8; 36]))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_water_transaction_valid() {
        let tx = WaterTransaction {
            from_agent: test_action().author,
            to_agent: other_agent(),
            liters: 10,
            credit_type: TransactionType::Transfer,
            timestamp: Timestamp::from_micros(0),
            source_hash: None,
        };
        let result = validate_create_water_transaction(test_action(), tx).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_water_transaction_sender_forgery_rejected() {
        let tx = WaterTransaction {
            from_agent: other_agent(),
            to_agent: test_action().author,
            liters: 10,
            credit_type: TransactionType::Transfer,
            timestamp: Timestamp::from_micros(0),
            source_hash: None,
        };
        let result = validate_create_water_transaction(test_action(), tx).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_usage_record_valid() {
        let usage = UsageRecord {
            agent: test_action().author,
            source_hash: ActionHash::from_raw_36(vec![9u8; 36]),
            liters_used: 10,
            usage_category: WaterClassification::Potable,
            recorded_at: Timestamp::from_micros(0),
            meter_reference: None,
        };
        let result = validate_create_usage_record(test_action(), usage).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_usage_record_agent_forgery_rejected() {
        let usage = UsageRecord {
            agent: other_agent(),
            source_hash: ActionHash::from_raw_36(vec![9u8; 36]),
            liters_used: 10,
            usage_category: WaterClassification::Potable,
            recorded_at: Timestamp::from_micros(0),
            meter_reference: None,
        };
        let result = validate_create_usage_record(test_action(), usage).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
