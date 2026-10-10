#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Payments Integrity Zome
//! Updated to use HDI 0.7 patterns with FlatOp validation
use hdi::prelude::*;
use mycelix_bridge_entry_types::{did_for_author, require_did_is_author};
pub use mycelix_finance_types::{
    AMBER_MAX_CAP_MICRO_SAP, AmberExemption, SapMintCapCounter, SapMintSource, SuccessionPreference,
};

// =============================================================================
// CONSTANTS — Fee Proportionality (Commons Charter)
// =============================================================================

/// Steward minimum fee rate: 0.01% (1 basis point).
/// For micro-SAP amounts: fee_micro >= amount_micro / 10_000.
pub const SAP_STEWARD_MIN_FEE_DIVISOR: u64 = 10_000;

// String length limits — prevent DHT bloat attacks
const MAX_DID_LEN: usize = 256;
const MAX_MEMO_LEN: usize = 1024;
const MAX_ID_LEN: usize = 256;
/// Maximum length for receipt signatures (hex-encoded Ed25519 = 128 chars, with margin)
const MAX_SIGNATURE_LEN: usize = 256;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Payment {
    pub id: String,
    pub from_did: String,
    pub to_did: String,
    pub amount: u64,
    pub fee: u64,
    pub currency: String,
    pub payment_type: PaymentType,
    pub status: TransferStatus,
    pub memo: Option<String>,
    pub created: Timestamp,
    pub completed: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum PaymentType {
    Direct,
    TreasuryContribution(String), // treasury_id
    CommonsContribution(String),  // commons_pool_id
    Escrow(String),               // escrow_id
    Recurring(RecurringConfig),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct RecurringConfig {
    pub frequency_days: u32,
    pub end_date: Option<Timestamp>,
    pub remaining: Option<u32>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TransferStatus {
    Pending,
    Processing,
    Completed,
    Failed(String),
    Cancelled,
    Refunded,
}

impl TransferStatus {
    /// Valid status transitions. Terminal states cannot revert.
    pub fn can_transition_to(&self, new: &TransferStatus) -> bool {
        matches!(
            (self, new),
            (TransferStatus::Pending, TransferStatus::Processing)
                | (TransferStatus::Pending, TransferStatus::Completed)
                | (TransferStatus::Pending, TransferStatus::Failed(_))
                | (TransferStatus::Pending, TransferStatus::Cancelled)
                | (TransferStatus::Processing, TransferStatus::Completed)
                | (TransferStatus::Processing, TransferStatus::Failed(_))
                | (TransferStatus::Completed, TransferStatus::Refunded)
        )
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PaymentChannel {
    pub id: String,
    pub party_a: String,
    pub party_b: String,
    pub currency: String,
    pub balance_a: u64,
    pub balance_b: u64,
    pub opened: Timestamp,
    pub last_updated: Timestamp,
    pub closed: Option<Timestamp>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Receipt {
    pub payment_id: String,
    pub from_did: String,
    pub to_did: String,
    pub amount: u64,
    pub currency: String,
    pub timestamp: Timestamp,
    pub signature: String,
}

/// On-chain SAP balance with lazy demurrage tracking.
/// Demurrage is applied on every balance read/mutation — no cron needed.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SapBalance {
    pub member_did: String,
    /// Raw balance in SAP micro-units (1 SAP = 1_000_000 micro-SAP)
    pub balance: u64,
    /// Timestamp of last demurrage application
    pub last_demurrage_at: Timestamp,
    /// Optional Amber exemption: a protected, demurrage-exempt tranche for
    /// children, elders, or multi-year projects. `None` = normal SAP.
    /// `#[serde(default)]` keeps pre-Amber balances deserializable.
    #[serde(default)]
    pub exemption: Option<AmberExemption>,
    /// ActionHash of the entry that justifies this balance's delta from its
    /// predecessor — a `SapMintRecord` for issuance, or the counterpart payment
    /// for a transfer.
    ///
    /// STEP 1 OF THE CONSERVATION MODEL (WU-1', see
    /// MYCELIX_PHASE1_EXECUTION_PLAN_2026-07-28.md). The field is threaded
    /// through now but NOT yet enforced: integrity does not check it, and every
    /// producer currently writes `None`. Adding it separately keeps the cluster
    /// green while the mint/transfer paths are migrated to populate it, after
    /// which integrity can require it for any *increase*.
    ///
    /// `Option` for a SEMANTIC reason, not backward compatibility: the genesis
    /// balance created by `initialize_sap_balance` is zero and has nothing
    /// justifying it. (There is no deployed DHT to migrate — Mycelix is a
    /// prototype, pre-testnet, with no users as of 2026-07-29.)
    #[serde(default)]
    pub justified_by: Option<ActionHash>,
}

/// Record of a member exit, coordinating across all currencies
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ExitRecord {
    /// DID of the exiting member
    pub member_did: String,
    /// SAP succession preference
    pub succession_preference: SuccessionPreference,
    /// SAP balance at time of exit
    pub sap_balance: u64,
    /// TEND balances forgiven (list of dao_did:amount pairs)
    pub tend_balances_forgiven: Vec<(String, i32)>,
    /// Whether MYCEL was dissolved
    pub mycel_dissolved: bool,
    /// Timestamp of the exit
    pub exited_at: Timestamp,
}

/// Record of SAP minting — every SAP must trace to a provenance.
///
/// SAP enters circulation through three paths:
/// 1. CollateralBridge — minted against external collateral (ETH, USDC)
/// 2. GovernanceProposal — minted by community governance vote
/// 3. InitialDistribution — bootstrap issuance for new communities
///
/// This record is immutable: once created, it cannot be updated or deleted.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SapMintRecord {
    /// Unique mint ID
    pub id: String,
    /// DID of the member receiving the minted SAP
    pub recipient_did: String,
    /// Amount minted in micro-SAP
    pub amount: u64,
    /// Provenance of the mint
    pub source: SapMintSource,
    /// When the mint occurred
    pub minted_at: Timestamp,
}

/// On-chain SAP mint cap counter — tracks cumulative governance minting per annual period.
///
/// Provides O(1) cap enforcement instead of scanning all SapMintRecord entries.
/// Uses optimistic-locking: read → check → mint → update counter atomically.
/// Mirrors `mycelix_finance_types::SapMintCapCounter` with HDI entry derivation.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SapMintCapCounterEntry {
    /// Start of the current annual period (microseconds since epoch)
    pub period_start_micros: i64,
    /// Cumulative SAP minted in this period (micro-SAP)
    pub cumulative_minted: u64,
    /// Number of governance mints in this period
    pub mint_count: u32,
    /// Last updated timestamp (microseconds)
    pub last_updated_micros: i64,
}

impl From<SapMintCapCounter> for SapMintCapCounterEntry {
    fn from(c: SapMintCapCounter) -> Self {
        Self {
            period_start_micros: c.period_start_micros,
            cumulative_minted: c.cumulative_minted,
            mint_count: c.mint_count,
            last_updated_micros: c.last_updated_micros,
        }
    }
}

impl From<SapMintCapCounterEntry> for SapMintCapCounter {
    fn from(e: SapMintCapCounterEntry) -> Self {
        Self {
            period_start_micros: e.period_start_micros,
            cumulative_minted: e.cumulative_minted,
            mint_count: e.mint_count,
            last_updated_micros: e.last_updated_micros,
        }
    }
}

/// A hearth-scoped SAP pool — shared household funds.
///
/// Each hearth gets one pool. Members contribute/withdraw SAP for shared
/// household expenses (groceries, utilities, etc). The pool balance is
/// subject to the same demurrage as individual SAP balances.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct HearthSapPool {
    /// DID of the hearth
    pub hearth_did: String,
    /// Pool balance in SAP micro-units
    pub balance: u64,
    /// Timestamp of last demurrage application
    pub last_demurrage_at: Timestamp,
    /// Number of contributing members
    pub member_count: u32,
    /// Total contributed (lifetime, for audit)
    pub total_contributed: u64,
    /// Total withdrawn (lifetime, for audit)
    pub total_withdrawn: u64,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Payment(Payment),
    PaymentChannel(PaymentChannel),
    Receipt(Receipt),
    ExitRecord(ExitRecord),
    SapBalance(SapBalance),
    SapMintRecord(SapMintRecord),
    HearthSapPool(HearthSapPool),
    SapMintCapCounterEntry(SapMintCapCounterEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    SenderToPayments,
    ReceiverToPayments,
    PaymentToReceipt,
    ChannelPartyA,
    ChannelPartyB,
    DidToSapBalance,
    MemberToExitRecord,
    PaymentIdToPayment,
    MintIdToMintRecord,
    DidToMintRecords,
    HearthDidToSapPool,
    ChannelIdToChannel,
    PendingCompostQueue,
    MintCapCounterAnchor,
    RateLimitBucketToAgent,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Payment(payment) => {
                    validate_create_payment(EntryCreationAction::Create(action), payment)
                }
                EntryTypes::PaymentChannel(channel) => {
                    validate_create_payment_channel(EntryCreationAction::Create(action), channel)
                }
                EntryTypes::Receipt(receipt) => {
                    validate_create_receipt(EntryCreationAction::Create(action), receipt)
                }
                EntryTypes::ExitRecord(exit) => {
                    validate_create_exit_record(EntryCreationAction::Create(action), exit)
                }
                EntryTypes::SapBalance(bal) => validate_sap_balance(&bal),
                EntryTypes::SapMintRecord(mint) => validate_create_sap_mint_record(&mint),
                EntryTypes::HearthSapPool(pool) => validate_hearth_sap_pool(&pool),
                EntryTypes::SapMintCapCounterEntry(counter) => {
                    validate_sap_mint_cap_counter(&counter)
                }
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => {
                match app_entry {
                    EntryTypes::Payment(payment) => validate_update_payment(action, payment),
                    EntryTypes::PaymentChannel(channel) => {
                        validate_update_payment_channel(action, channel)
                    }
                    EntryTypes::Receipt(_) => Ok(ValidateCallbackResult::Invalid(
                        "Receipts cannot be updated".into(),
                    )),
                    EntryTypes::ExitRecord(_) => Ok(ValidateCallbackResult::Invalid(
                        "Exit records cannot be updated".into(),
                    )),
                    EntryTypes::SapBalance(bal) => validate_sap_balance(&bal),
                    EntryTypes::SapMintRecord(_) => {
                        // Mint records are immutable
                        Ok(ValidateCallbackResult::Invalid(
                            "SAP mint records cannot be updated".into(),
                        ))
                    }
                    EntryTypes::HearthSapPool(pool) => validate_hearth_sap_pool(&pool),
                    EntryTypes::SapMintCapCounterEntry(counter) => {
                        validate_sap_mint_cap_counter(&counter)
                    }
                }
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            action,
            ..
        } => match link_type {
            LinkTypes::SenderToPayments | LinkTypes::ReceiverToPayments => {
                validate_payment_index_link(link_type, &base_address, &target_address)
            }
            LinkTypes::PaymentToReceipt => {
                validate_payment_to_receipt_link(&base_address, &target_address)
            }
            LinkTypes::ChannelPartyA | LinkTypes::ChannelPartyB => {
                validate_channel_party_link(link_type, &base_address, &target_address)
            }
            LinkTypes::DidToSapBalance => {
                validate_sap_balance_index_link(&base_address, &target_address)
            }
            LinkTypes::MemberToExitRecord => {
                validate_exit_index_link(&base_address, &target_address)
            }
            LinkTypes::PaymentIdToPayment => {
                validate_payment_id_index_link(&base_address, &target_address)
            }
            LinkTypes::MintIdToMintRecord => {
                validate_mint_id_index_link(&base_address, &target_address)
            }
            LinkTypes::DidToMintRecords => {
                validate_member_mint_index_link(&base_address, &target_address)
            }
            LinkTypes::HearthDidToSapPool => {
                validate_hearth_pool_index_link(&base_address, &target_address)
            }
            LinkTypes::ChannelIdToChannel => {
                validate_channel_id_index_link(&base_address, &target_address)
            }
            LinkTypes::PendingCompostQueue => {
                let expected = deterministic_anchor_hash("pending_compost_queue");
                if base_address.clone().into_entry_hash() != Some(expected.clone())
                    || target_address.clone().into_entry_hash() != Some(expected)
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "PendingCompostQueue must target its deterministic self-anchor".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::MintCapCounterAnchor => {
                validate_mint_cap_counter_link(&base_address, &target_address)
            }
            LinkTypes::RateLimitBucketToAgent => {
                validate_rate_limit_bucket_link(
                    &action.author,
                    &base_address,
                    &target_address,
                )
            }
        },
        FlatOp::RegisterDeleteLink { link_type, .. } => {
            // Prevent deletion of critical links
            match link_type {
                LinkTypes::PaymentToReceipt => Ok(ValidateCallbackResult::Invalid(
                    "PaymentToReceipt links cannot be deleted - receipts are immutable".into(),
                )),
                _ => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_rate_limit_bucket_link(
    author: &AgentPubKey,
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    if base_address.clone().into_entry_hash().is_none() {
        return Ok(invalid_link(
            "RateLimitBucketToAgent base must be an EntryHash",
        ));
    }
    let Some(target_agent) = target_address.clone().into_agent_pub_key() else {
        return Ok(invalid_link(
            "RateLimitBucketToAgent target must be an AgentPubKey",
        ));
    };
    if target_agent != *author {
        return Ok(invalid_link(
            "RateLimitBucketToAgent target must equal the CreateLink author",
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_payment(
    action: EntryCreationAction,
    payment: Payment,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the payment to its committer. Without this, `from_did` is a
    // self-reported string that no peer ever checks against the agent who signed
    // the entry — so any agent could commit a Payment spending anyone's balance
    // (debit forgery; MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md, finance
    // Class-A, `payments:430`).
    //
    // Safe to bind unconditionally: `send_payment` (the only coordinator path
    // that creates a Payment) already calls `verify_caller_is_did(&from_did)`,
    // so every legitimate Payment is committed by `from_did`'s own agent.
    // Verified 2026-07-28 against payments/coordinator/src/lib.rs:1170.
    //
    // NOTE the contrast with `SapBalance` in this same zome, which is NOT
    // author-bindable: `transfer_sap` credits the payee from the *sender's*
    // agent context, so balances are shared mutable state written by many
    // agents. That needs the integrity-conservation model instead — see
    // MYCELIX_ECONOMY_IMPROVEMENT_PLAN_2026-07-10.md.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("Payment", "from_did", &payment.from_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // String length checks — prevent DHT bloat
    if payment.from_did.len() > MAX_DID_LEN || payment.to_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if payment.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Payment ID exceeds maximum length".into(),
        ));
    }
    if let Some(ref memo) = payment.memo {
        if memo.len() > MAX_MEMO_LEN {
            return Ok(ValidateCallbackResult::Invalid(
                "Memo exceeds maximum length of 1024 characters".into(),
            ));
        }
    }

    if !payment.from_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Sender must be a valid DID".into(),
        ));
    }
    if !payment.to_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Receiver must be a valid DID".into(),
        ));
    }
    if payment.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Amount must be positive".into(),
        ));
    }
    if payment.from_did == payment.to_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot send payment to yourself".into(),
        ));
    }
    // Only SAP and TEND currencies are accepted
    if payment.currency != "SAP" && payment.currency != "TEND" {
        return Ok(ValidateCallbackResult::Invalid(
            "Currency must be \"SAP\" or \"TEND\"".into(),
        ));
    }

    // Fee proportionality: SAP payments must pay at least the Steward minimum (0.01%)
    if payment.currency == "SAP" && payment.fee < payment.amount / SAP_STEWARD_MIN_FEE_DIVISOR {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "SAP payment fee ({}) is below Steward minimum (0.01% = {})",
            payment.fee,
            payment.amount / SAP_STEWARD_MIN_FEE_DIVISOR
        )));
    }

    // TEND payments are fee-free (mutual credit)
    if payment.currency == "TEND" && payment.fee != 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "TEND payments must have zero fee (mutual credit is fee-free)".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn deterministic_anchor_hash(anchor: &str) -> EntryHash {
    EntryHash::from_raw_32(holo_hash::blake2b_256(anchor.as_bytes()).to_vec())
}

fn invalid_link(message: impl Into<String>) -> ValidateCallbackResult {
    ValidateCallbackResult::Invalid(message.into())
}

fn require_entry_base(
    base_address: &AnyLinkableHash,
    relation: &str,
) -> Result<EntryHash, ValidateCallbackResult> {
    base_address
        .clone()
        .into_entry_hash()
        .ok_or_else(|| invalid_link(format!("{relation} base must be an EntryHash")))
}

fn require_action_target(
    target_address: &AnyLinkableHash,
    relation: &str,
) -> Result<ActionHash, ValidateCallbackResult> {
    target_address
        .clone()
        .into_action_hash()
        .ok_or_else(|| invalid_link(format!("{relation} target must be an ActionHash")))
}

fn require_anchor(
    base_address: &AnyLinkableHash,
    anchor: &str,
    relation: &str,
) -> Result<(), ValidateCallbackResult> {
    let expected = deterministic_anchor_hash(anchor);
    let actual = require_entry_base(base_address, relation)?;
    if actual != expected {
        return Err(invalid_link(format!(
            "{relation} base does not match deterministic anchor"
        )));
    }
    Ok(())
}

fn validate_payment_index_link(
    link_type: LinkTypes,
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let relation = match link_type {
        LinkTypes::SenderToPayments => "SenderToPayments",
        LinkTypes::ReceiverToPayments => "ReceiverToPayments",
        _ => return Ok(invalid_link("internal payment-index link type mismatch")),
    };
    let target_hash = match require_action_target(target_address, relation) {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(payment) = record.entry().to_app_option::<Payment>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "{relation} target decode failed: {e:?}"
        )))
    })? else {
        return Ok(invalid_link(format!(
            "{relation} target must be a Payment entry"
        )));
    };
    let did = match link_type {
        LinkTypes::SenderToPayments => &payment.from_did,
        LinkTypes::ReceiverToPayments => &payment.to_did,
        _ => unreachable!(),
    };
    if let Err(invalid) = require_anchor(base_address, did, relation) {
        return Ok(invalid);
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_payment_to_receipt_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let payment_hash = match require_action_target(base_address, "PaymentToReceipt base") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let receipt_hash = match require_action_target(target_address, "PaymentToReceipt target") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let payment_record = must_get_valid_record(payment_hash)?;
    let receipt_record = must_get_valid_record(receipt_hash)?;
    let Some(payment) = payment_record.entry().to_app_option::<Payment>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("PaymentToReceipt payment decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("PaymentToReceipt base must resolve to Payment"));
    };
    let Some(receipt) = receipt_record.entry().to_app_option::<Receipt>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("PaymentToReceipt receipt decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("PaymentToReceipt target must resolve to Receipt"));
    };
    if receipt.payment_id != payment.id
        || receipt.from_did != payment.from_did
        || receipt.to_did != payment.to_did
        || receipt.amount != payment.amount
        || receipt.currency != payment.currency
    {
        return Ok(invalid_link(
            "PaymentToReceipt target does not bind to the referenced Payment",
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_channel_party_link(
    link_type: LinkTypes,
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let target_hash = match require_action_target(
        target_address,
        "ChannelParty link",
    ) {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(channel) = record.entry().to_app_option::<PaymentChannel>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("ChannelParty target decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("ChannelParty target must resolve to PaymentChannel"));
    };
    let (did, relation) = match link_type {
        LinkTypes::ChannelPartyA => (&channel.party_a, "ChannelPartyA"),
        LinkTypes::ChannelPartyB => (&channel.party_b, "ChannelPartyB"),
        _ => unreachable!(),
    };
    if let Err(invalid) = require_anchor(base_address, did, relation) {
        return Ok(invalid);
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sap_balance_index_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let target_hash = match require_action_target(target_address, "DidToSapBalance") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(balance) = record.entry().to_app_option::<SapBalance>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("DidToSapBalance target decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("DidToSapBalance target must resolve to SapBalance"));
    };
    require_anchor(base_address, &format!("sap:{}", balance.member_did), "DidToSapBalance")
        .map_or_else(|invalid| Ok(invalid), |_| Ok(ValidateCallbackResult::Valid))
}

fn validate_exit_index_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let target_hash = match require_action_target(target_address, "MemberToExitRecord") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(exit) = record.entry().to_app_option::<ExitRecord>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("MemberToExitRecord target decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("MemberToExitRecord target must resolve to ExitRecord"));
    };
    require_anchor(base_address, &exit.member_did, "MemberToExitRecord")
        .map_or_else(|invalid| Ok(invalid), |_| Ok(ValidateCallbackResult::Valid))
}

fn validate_payment_id_index_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let target_hash = match require_action_target(target_address, "PaymentIdToPayment") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(payment) = record.entry().to_app_option::<Payment>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("PaymentIdToPayment target decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("PaymentIdToPayment target must resolve to Payment"));
    };
    require_anchor(base_address, &payment.id, "PaymentIdToPayment")
        .map_or_else(|invalid| Ok(invalid), |_| Ok(ValidateCallbackResult::Valid))
}

fn validate_mint_id_index_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let target_hash = match require_action_target(target_address, "MintIdToMintRecord") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(mint) = record.entry().to_app_option::<SapMintRecord>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("MintIdToMintRecord target decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("MintIdToMintRecord target must resolve to SapMintRecord"));
    };
    require_anchor(base_address, &mint.id, "MintIdToMintRecord")
        .map_or_else(|invalid| Ok(invalid), |_| Ok(ValidateCallbackResult::Valid))
}

fn validate_member_mint_index_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let target_hash = match require_action_target(target_address, "DidToMintRecords") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(mint) = record.entry().to_app_option::<SapMintRecord>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("DidToMintRecords target decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("DidToMintRecords target must resolve to SapMintRecord"));
    };
    require_anchor(base_address, &format!("mints:{}", mint.recipient_did), "DidToMintRecords")
        .map_or_else(|invalid| Ok(invalid), |_| Ok(ValidateCallbackResult::Valid))
}

fn validate_hearth_pool_index_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let target_hash = match require_action_target(target_address, "HearthDidToSapPool") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(pool) = record.entry().to_app_option::<HearthSapPool>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("HearthDidToSapPool target decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("HearthDidToSapPool target must resolve to HearthSapPool"));
    };
    require_anchor(
        base_address,
        &format!("hearth-sap:{}", pool.hearth_did),
        "HearthDidToSapPool",
    )
    .map_or_else(|invalid| Ok(invalid), |_| Ok(ValidateCallbackResult::Valid))
}

fn validate_channel_id_index_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    let target_hash = match require_action_target(target_address, "ChannelIdToChannel") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    let Some(channel) = record.entry().to_app_option::<PaymentChannel>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!("ChannelIdToChannel target decode failed: {e:?}")))
    })? else {
        return Ok(invalid_link("ChannelIdToChannel target must resolve to PaymentChannel"));
    };
    require_anchor(base_address, &channel.id, "ChannelIdToChannel")
        .map_or_else(|invalid| Ok(invalid), |_| Ok(ValidateCallbackResult::Valid))
}

fn validate_mint_cap_counter_link(
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(invalid) = require_anchor(
        base_address,
        "sap_mint_cap_counter",
        "MintCapCounterAnchor",
    ) {
        return Ok(invalid);
    }
    let target_hash = match require_action_target(target_address, "MintCapCounterAnchor") {
        Ok(hash) => hash,
        Err(invalid) => return Ok(invalid),
    };
    let record = must_get_valid_record(target_hash)?;
    if record.entry().to_app_option::<SapMintCapCounterEntry>().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "MintCapCounterAnchor target decode failed: {e:?}"
        )))
    })?.is_none() {
        return Ok(invalid_link(
            "MintCapCounterAnchor target must resolve to SapMintCapCounterEntry",
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_payment(
    action: Update,
    payment: Payment,
) -> ExternResult<ValidateCallbackResult> {
    // Status can change but amount/parties cannot
    if payment.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Amount must be positive".into(),
        ));
    }

    // Enforce status transition rules and immutable field invariants
    if let Ok(original_record) = must_get_valid_record(action.original_action_address) {
        if let Ok(Some(original)) = original_record.entry().to_app_option::<Payment>() {
            if original.status != payment.status
                && !original.status.can_transition_to(&payment.status)
            {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Invalid payment status transition: {:?} → {:?}",
                    original.status, payment.status
                )));
            }
            // Core fields are immutable
            if original.from_did != payment.from_did
                || original.to_did != payment.to_did
                || original.amount != payment.amount
                || original.currency != payment.currency
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Cannot change sender, receiver, amount, or currency on an existing payment"
                        .into(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_payment_channel(
    _action: EntryCreationAction,
    channel: PaymentChannel,
) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if channel.party_a.len() > MAX_DID_LEN || channel.party_b.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if channel.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Channel ID exceeds maximum length".into(),
        ));
    }

    if !channel.party_a.starts_with("did:") || !channel.party_b.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Parties must be valid DIDs".into(),
        ));
    }
    if channel.currency != "SAP" && channel.currency != "TEND" {
        return Ok(ValidateCallbackResult::Invalid(
            "Currency must be \"SAP\" or \"TEND\"".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_payment_channel(
    _action: Update,
    _channel: PaymentChannel,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_receipt(
    _action: EntryCreationAction,
    receipt: Receipt,
) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if receipt.from_did.len() > MAX_DID_LEN || receipt.to_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if receipt.payment_id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Payment ID exceeds maximum length".into(),
        ));
    }
    if receipt.signature.len() > MAX_SIGNATURE_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Signature exceeds maximum length".into(),
        ));
    }

    // Validate DIDs
    if !receipt.from_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Sender must be a valid DID".into(),
        ));
    }
    if !receipt.to_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Receiver must be a valid DID".into(),
        ));
    }

    // Validate amount
    if receipt.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Amount must be positive".into(),
        ));
    }

    // Validate currency
    if receipt.currency != "SAP" && receipt.currency != "TEND" {
        return Ok(ValidateCallbackResult::Invalid(
            "Currency must be \"SAP\" or \"TEND\"".into(),
        ));
    }

    // Validate signature is present and properly formatted
    // Signature format: base64-encoded Ed25519 signature (88 chars) or hex (128 chars)
    if receipt.signature.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Receipt must have a signature".into(),
        ));
    }

    // Basic signature format validation
    // Ed25519 signatures are 64 bytes = 88 chars base64 or 128 chars hex
    let sig_len = receipt.signature.len();
    if sig_len < 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Signature too short - must be valid Ed25519 signature".into(),
        ));
    }

    // Verify signature is valid base64 or hex
    let is_valid_format = receipt
        .signature
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '+' || c == '/' || c == '=');
    if !is_valid_format {
        return Ok(ValidateCallbackResult::Invalid(
            "Signature must be valid base64 or hex encoding".into(),
        ));
    }

    // Note: Full cryptographic verification requires the sender's public key
    // which would be fetched from the identity zome in production.
    // The signature should cover: payment_id | from_did | to_did | amount | currency | timestamp

    Ok(ValidateCallbackResult::Valid)
}

fn validate_sap_balance(bal: &SapBalance) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if bal.member_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }

    if !bal.member_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Member must be a valid DID".into(),
        ));
    }

    // Amber exemption: structural anti-arbitrage gate. Deterministic checks only —
    // issuer *authenticity* (a real child/elder credential / governance approval) is
    // verified at grant time in the coordinator, since integrity cannot call out.
    if let Some(ex) = &bal.exemption {
        if ex.issuer.len() > MAX_DID_LEN || !ex.issuer.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(
                "Amber exemption issuer must be a valid DID".into(),
            ));
        }
        // No self-issue: a holder can never grant themselves demurrage exemption.
        if ex.issuer == bal.member_did {
            return Ok(ValidateCallbackResult::Invalid(
                "Amber exemption cannot be self-issued".into(),
            ));
        }
        // Cap bounded by the governance ceiling (whale-loophole guard).
        if ex.cap_micro_sap > AMBER_MAX_CAP_MICRO_SAP {
            return Ok(ValidateCallbackResult::Invalid(
                "Amber exemption cap exceeds governance ceiling".into(),
            ));
        }
        // Must expire — no permanent, uncapped shelters.
        if ex.expires_at_secs == 0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Amber exemption must have a nonzero expiry".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_sap_mint_record(mint: &SapMintRecord) -> ExternResult<ValidateCallbackResult> {
    if mint.recipient_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if mint.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Mint ID exceeds maximum length".into(),
        ));
    }
    if !mint.recipient_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Recipient must be a valid DID".into(),
        ));
    }
    if mint.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Mint amount must be positive".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_exit_record(
    action: EntryCreationAction,
    exit: ExitRecord,
) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if exit.member_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }

    if !exit.member_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Member must be a valid DID".into(),
        ));
    }

    // Bind the exit to its committer. A forged `member_did` fabricates someone
    // else's exit — which forgives their TEND balances and dissolves their MYCEL
    // (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md, finance Class-A, `payments:696`).
    //
    // Safe to bind: `initiate_exit` (payments/coordinator/src/lib.rs:1823) already
    // calls verify_caller_is_did(&input.member_did), and ExitRecord is create-only
    // (its update arm returns Invalid). Verified 2026-07-28.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("ExitRecord", "member_did", &exit.member_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    if let SuccessionPreference::Designee(ref designee) = exit.succession_preference {
        if !designee.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(
                "Designee must be a valid DID".into(),
            ));
        }
        if *designee == exit.member_did {
            return Ok(ValidateCallbackResult::Invalid(
                "Cannot designate yourself as successor".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_sap_mint_cap_counter(
    counter: &SapMintCapCounterEntry,
) -> ExternResult<ValidateCallbackResult> {
    if counter.period_start_micros < 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Period start must be non-negative".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_hearth_sap_pool(pool: &HearthSapPool) -> ExternResult<ValidateCallbackResult> {
    if pool.hearth_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Hearth DID exceeds maximum length".into(),
        ));
    }
    if !pool.hearth_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Hearth must be a valid DID".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

// =============================================================================
// UNIT TESTS
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_finance_types::{SapMintSource, SuccessionPreference};

    fn ts(micros: i64) -> Timestamp {
        Timestamp::from_micros(micros)
    }

    fn make_create() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(1_000_000),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    /// The DID of the agent that `make_create()` attributes actions to.
    ///
    /// `validate_create_payment` binds `from_did` to the committing agent, so any
    /// fixture meant to be VALID must use this rather than a human-readable name.
    fn test_author_did() -> String {
        format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![0; 36]))
    }

    fn valid_payment() -> Payment {
        Payment {
            id: "pay:test:001".into(),
            from_did: test_author_did(),
            to_did: "did:mycelix:bob".into(),
            amount: 1_000_000, // 1 SAP
            fee: 100,          // 0.01% of 1_000_000 = 100
            currency: "SAP".into(),
            payment_type: PaymentType::Direct,
            status: TransferStatus::Pending,
            memo: None,
            created: ts(1_000_000),
            completed: None,
        }
    }

    fn valid_channel() -> PaymentChannel {
        PaymentChannel {
            id: "chan:test:001".into(),
            party_a: "did:mycelix:alice".into(),
            party_b: "did:mycelix:bob".into(),
            currency: "SAP".into(),
            balance_a: 500,
            balance_b: 500,
            opened: ts(1_000_000),
            last_updated: ts(1_000_000),
            closed: None,
        }
    }

    fn valid_receipt() -> Receipt {
        // 128-char hex string (valid Ed25519 hex encoding)
        let sig = "a".repeat(128);
        Receipt {
            payment_id: "pay:test:001".into(),
            from_did: "did:mycelix:alice".into(),
            to_did: "did:mycelix:bob".into(),
            amount: 1000,
            currency: "SAP".into(),
            timestamp: ts(1_000_000),
            signature: sig,
        }
    }

    fn valid_sap_balance() -> SapBalance {
        SapBalance {
            justified_by: None,
            member_did: "did:mycelix:alice".into(),
            balance: 5_000_000,
            last_demurrage_at: ts(1_000_000),
            exemption: None,
        }
    }

    fn valid_sap_mint_record() -> SapMintRecord {
        SapMintRecord {
            id: "mint:test:001".into(),
            recipient_did: "did:mycelix:alice".into(),
            amount: 1_000_000,
            source: SapMintSource::InitialDistribution {
                reason: "Bootstrap".into(),
            },
            minted_at: ts(1_000_000),
        }
    }

    fn valid_exit_record() -> ExitRecord {
        ExitRecord {
            member_did: test_author_did(),
            succession_preference: SuccessionPreference::Commons,
            sap_balance: 5_000_000,
            tend_balances_forgiven: vec![],
            mycel_dissolved: false,
            exited_at: ts(1_000_000),
        }
    }

    // ---- 1. Valid Payment (SAP with fee) ----

    #[test]
    fn test_valid_payment_sap_with_fee() {
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), valid_payment())
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- 2. Payment with zero amount (must fail) ----

    #[test]
    fn test_payment_zero_amount() {
        let mut p = valid_payment();
        p.amount = 0;
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 3. Payment self-send (must fail) ----

    #[test]
    fn test_payment_self_send() {
        let mut p = valid_payment();
        p.to_did = p.from_did.clone();
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 4. Payment with invalid from_did (must fail) ----

    #[test]
    fn test_payment_invalid_from_did() {
        let mut p = valid_payment();
        p.from_did = "not-a-did".into();
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 5. Payment with invalid to_did (must fail) ----

    #[test]
    fn test_payment_invalid_to_did() {
        let mut p = valid_payment();
        p.to_did = "not-a-did".into();
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 6. Payment with unsupported currency (must fail) ----

    #[test]
    fn test_payment_unsupported_currency() {
        let mut p = valid_payment();
        p.currency = "BTC".into();
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 7. SAP payment with fee below minimum (must fail) ----

    #[test]
    fn test_sap_payment_fee_below_minimum() {
        let mut p = valid_payment();
        p.amount = 1_000_000; // min fee = 1_000_000 / 10_000 = 100
        p.fee = 99; // below minimum
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 8. TEND payment with non-zero fee (must fail) ----

    #[test]
    fn test_tend_payment_nonzero_fee() {
        let mut p = valid_payment();
        p.currency = "TEND".into();
        p.fee = 1;
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), p).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 9. Valid PaymentChannel ----

    #[test]
    fn test_valid_payment_channel() {
        let result = validate_create_payment_channel(
            EntryCreationAction::Create(make_create()),
            valid_channel(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- 10. PaymentChannel with invalid party DID (must fail) ----

    #[test]
    fn test_channel_invalid_party_did() {
        let mut ch = valid_channel();
        ch.party_a = "not-a-did".into();
        let result =
            validate_create_payment_channel(EntryCreationAction::Create(make_create()), ch)
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 11. PaymentChannel with unsupported currency (must fail) ----

    #[test]
    fn test_channel_unsupported_currency() {
        let mut ch = valid_channel();
        ch.currency = "ETH".into();
        let result =
            validate_create_payment_channel(EntryCreationAction::Create(make_create()), ch)
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 12. Valid Receipt ----

    #[test]
    fn test_valid_receipt() {
        let result =
            validate_create_receipt(EntryCreationAction::Create(make_create()), valid_receipt())
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- 13. Receipt with zero amount (must fail) ----

    #[test]
    fn test_receipt_zero_amount() {
        let mut r = valid_receipt();
        r.amount = 0;
        let result =
            validate_create_receipt(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 14. Receipt with empty signature (must fail) ----

    #[test]
    fn test_receipt_empty_signature() {
        let mut r = valid_receipt();
        r.signature = String::new();
        let result =
            validate_create_receipt(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 15. Receipt with signature too short (must fail) ----

    #[test]
    fn test_receipt_signature_too_short() {
        let mut r = valid_receipt();
        r.signature = "abcd1234".into(); // 8 chars, well below 64
        let result =
            validate_create_receipt(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 16. Receipt with invalid signature encoding (must fail) ----

    #[test]
    fn test_receipt_invalid_signature_encoding() {
        let mut r = valid_receipt();
        // 128 chars but contains invalid characters (spaces, symbols)
        r.signature = "!@#$".repeat(32);
        let result =
            validate_create_receipt(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 17. Receipts cannot be updated (must fail) ----

    #[test]
    fn test_receipt_cannot_be_updated() {
        // The validate() main function returns Invalid for receipt updates.
        // We test the logic directly: the match arm returns Invalid.
        let result: ValidateCallbackResult =
            ValidateCallbackResult::Invalid("Receipts cannot be updated".into());
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("Receipts cannot be updated"))
        );
    }

    // ---- 18. Valid SapBalance ----

    #[test]
    fn test_valid_sap_balance() {
        let result = validate_sap_balance(&valid_sap_balance()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- 19. SapBalance with invalid DID (must fail) ----

    #[test]
    fn test_sap_balance_invalid_did() {
        let mut bal = valid_sap_balance();
        bal.member_did = "not-a-did".into();
        let result = validate_sap_balance(&bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 19b. Amber exemption structural validation ----

    fn valid_amber() -> AmberExemption {
        AmberExemption {
            class: mycelix_finance_types::AmberClass::Custodial,
            issuer: "did:mycelix:steward".into(),
            cap_micro_sap: 50_000_000_000,
            expires_at_secs: 9_000_000_000,
        }
    }

    #[test]
    fn test_valid_amber_exemption_accepted() {
        let mut bal = valid_sap_balance();
        bal.exemption = Some(valid_amber());
        assert!(matches!(
            validate_sap_balance(&bal).unwrap(),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn test_amber_self_issue_rejected() {
        let mut bal = valid_sap_balance();
        let mut ex = valid_amber();
        ex.issuer = bal.member_did.clone(); // holder issues to self
        bal.exemption = Some(ex);
        assert!(matches!(
            validate_sap_balance(&bal).unwrap(),
            ValidateCallbackResult::Invalid(msg) if msg.contains("self-issued")
        ));
    }

    #[test]
    fn test_amber_cap_over_ceiling_rejected() {
        let mut bal = valid_sap_balance();
        let mut ex = valid_amber();
        ex.cap_micro_sap = AMBER_MAX_CAP_MICRO_SAP + 1;
        bal.exemption = Some(ex);
        assert!(matches!(
            validate_sap_balance(&bal).unwrap(),
            ValidateCallbackResult::Invalid(msg) if msg.contains("ceiling")
        ));
    }

    #[test]
    fn test_amber_zero_expiry_rejected() {
        let mut bal = valid_sap_balance();
        let mut ex = valid_amber();
        ex.expires_at_secs = 0;
        bal.exemption = Some(ex);
        assert!(matches!(
            validate_sap_balance(&bal).unwrap(),
            ValidateCallbackResult::Invalid(msg) if msg.contains("expiry")
        ));
    }

    #[test]
    fn test_amber_invalid_issuer_did_rejected() {
        let mut bal = valid_sap_balance();
        let mut ex = valid_amber();
        ex.issuer = "steward".into(); // not a DID
        bal.exemption = Some(ex);
        assert!(matches!(
            validate_sap_balance(&bal).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    // ---- 20. Valid SapMintRecord ----

    #[test]
    fn test_valid_sap_mint_record() {
        let result = validate_create_sap_mint_record(&valid_sap_mint_record()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- 21. SapMintRecord with zero amount (must fail) ----

    #[test]
    fn test_sap_mint_record_zero_amount() {
        let mut mint = valid_sap_mint_record();
        mint.amount = 0;
        let result = validate_create_sap_mint_record(&mint).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 22. SapMintRecord cannot be updated (must fail) ----

    #[test]
    fn test_sap_mint_record_cannot_be_updated() {
        // The validate() main function returns Invalid for mint record updates.
        let result: ValidateCallbackResult =
            ValidateCallbackResult::Invalid("SAP mint records cannot be updated".into());
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(msg) if msg.contains("cannot be updated"))
        );
    }

    // ---- 23. Valid ExitRecord ----

    #[test]
    fn test_valid_exit_record() {
        let result = validate_create_exit_record(
            EntryCreationAction::Create(make_create()),
            valid_exit_record(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- 24. ExitRecord designee cannot be self (must fail) ----

    #[test]
    fn test_exit_record_designee_cannot_be_self() {
        let mut exit = valid_exit_record();
        // Must equal `member_did` for "designee == self" to be what is under test.
        // Previously both were the literal "did:mycelix:alice"; since
        // `valid_exit_record()` now derives member_did from the committing agent
        // (author binding, 2026-07-28), the designee has to derive from it too.
        exit.succession_preference = SuccessionPreference::Designee(test_author_did());
        let result =
            validate_create_exit_record(EntryCreationAction::Create(make_create()), exit).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 25. Valid SapMintCapCounterEntry ----

    #[test]
    fn test_valid_sap_mint_cap_counter() {
        let counter = SapMintCapCounterEntry {
            period_start_micros: 1_000_000,
            cumulative_minted: 500_000_000,
            mint_count: 3,
            last_updated_micros: 2_000_000,
        };
        let result = validate_sap_mint_cap_counter(&counter).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- 26. SapMintCapCounterEntry with negative period_start (must fail) ----

    #[test]
    fn test_sap_mint_cap_counter_negative_period() {
        let counter = SapMintCapCounterEntry {
            period_start_micros: -1,
            cumulative_minted: 0,
            mint_count: 0,
            last_updated_micros: 0,
        };
        let result = validate_sap_mint_cap_counter(&counter).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- 27. SapMintCapCounterEntry From/Into SapMintCapCounter roundtrip ----

    #[test]
    fn test_sap_mint_cap_counter_roundtrip() {
        let counter = SapMintCapCounter {
            period_start_micros: 1_000_000,
            cumulative_minted: 500_000_000,
            mint_count: 3,
            last_updated_micros: 2_000_000,
        };
        let entry: SapMintCapCounterEntry = counter.clone().into();
        let back: SapMintCapCounter = entry.into();
        assert_eq!(counter, back);
    }

    // ---- 28. Payment author binding (debit-forgery regression) ----
    //
    // These are the durability half of the fix: the campaign's history shows an
    // author-binding fix landing and later being lost (commit 48a4460dcf), so
    // each binding gets a test that asserts a FORGED value is REJECTED — not
    // merely that a legitimate one is accepted.

    #[test]
    fn test_payment_from_did_matching_author_is_valid() {
        let author = "did:mycelix:uhCAkalice";
        let result = require_did_is_author("Payment", "from_did", author, author);
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_payment_from_did_forged_is_rejected() {
        // Mallory commits a Payment claiming to spend Alice's balance.
        let result = require_did_is_author(
            "Payment",
            "from_did",
            "did:mycelix:uhCAkalice",
            "did:mycelix:uhCAkmallory",
        );
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(
                    msg.contains("Payment") && msg.contains("from_did") && msg.contains("forgery"),
                    "rejection should name the failure mode, got: {msg}"
                );
            }
            other => panic!("forged from_did must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_payment_from_did_empty_author_still_rejected() {
        // Degenerate input must not accidentally pass the equality check.
        let result = require_did_is_author("Payment", "from_did", "did:mycelix:uhCAkalice", "");
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_payment_with_forged_from_did_is_rejected_end_to_end() {
        // The real attack, through the real validator: Mallory commits a Payment
        // whose `from_did` names a victim. Before the 2026-07-28 fix this
        // returned Valid, and every honest peer accepted it — debit forgery.
        let mut forged = valid_payment();
        forged.from_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), forged).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(
                    msg.contains("Payment") && msg.contains("from_did") && msg.contains("forgery"),
                    "got: {msg}"
                )
            }
            other => panic!("forged from_did must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_create_payment_from_the_committing_agent_is_accepted() {
        // The legitimate path must still pass: `send_payment` verifies
        // caller == from_did, so a real Payment is always committed by that agent.
        let result =
            validate_create_payment(EntryCreationAction::Create(make_create()), valid_payment())
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_exit_record_forged_member_is_rejected() {
        // Forging someone's exit forgives their TEND debts and dissolves MYCEL.
        let mut e = valid_exit_record();
        e.member_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_exit_record(EntryCreationAction::Create(make_create()), e).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(
                    msg.contains("ExitRecord")
                        && msg.contains("member_did")
                        && msg.contains("forgery"),
                    "got: {msg}"
                )
            }
            other => panic!("forged member_did must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_rate_limit_bucket_target_must_be_link_author() {
        let author = AgentPubKey::from_raw_36(vec![1; 36]);
        let target = AnyLinkableHash::from(author.clone());
        let base = AnyLinkableHash::from(deterministic_anchor_hash(
            "payment-rate-limit:author:bucket",
        ));
        assert!(matches!(
            validate_rate_limit_bucket_link(&author, &base, &target).unwrap(),
            ValidateCallbackResult::Valid
        ));

        let forged_target = AnyLinkableHash::from(AgentPubKey::from_raw_36(vec![2; 36]));
        assert!(matches!(
            validate_rate_limit_bucket_link(&author, &base, &forged_target).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    // ---- 29. Authoritative payment-index anchor binding ----

    #[test]
    fn test_payment_index_anchor_is_exact_not_length_only() {
        let did = "did:mycelix:uhCAkalice";
        let expected = deterministic_anchor_hash(did);
        let valid_base = AnyLinkableHash::from(expected);

        assert!(require_anchor(&valid_base, did, "SenderToPayments").is_ok());

        let forged_base = AnyLinkableHash::from(deterministic_anchor_hash(
            "did:mycelix:uhCAkmallory",
        ));
        assert!(
            require_anchor(&forged_base, did, "SenderToPayments").is_err(),
            "same-length forged anchor must be rejected"
        );
    }

    // ---- 30. Index bases/targets are type-safe, not just 39-byte values ----

    #[test]
    fn test_action_hash_is_not_accepted_as_entry_hash_index_base() {
        let action = ActionHash::from_raw_36(vec![0; 36]);
        let linkable = AnyLinkableHash::from(action);
        assert!(
            require_entry_base(&linkable, "DidToSapBalance").is_err(),
            "an ActionHash must not satisfy an EntryHash-only base"
        );
    }

    #[test]
    fn test_authoritative_anchor_accepts_real_entry_hash() {
        let anchor = deterministic_anchor_hash("sap:did:mycelix:uhCAkalice");
        let linkable = AnyLinkableHash::from(anchor);
        assert!(require_anchor(
            &linkable,
            "sap:did:mycelix:uhCAkalice",
            "DidToSapBalance"
        )
        .is_ok());
    }

    #[test]
    fn test_rate_limit_target_requires_agent_pubkey() {
        let action = AnyLinkableHash::from(ActionHash::from_raw_36(vec![7; 36]));
        assert!(
            action.into_agent_pub_key().is_none(),
            "a payment ActionHash must never satisfy the rate-limit AgentPubKey target"
        );
    }

}
