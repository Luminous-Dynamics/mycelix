// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdk::prelude::*;
use listings_integrity::{Listing, ListingStatus};
use mycelix_common::{link_queries, remote_calls, time};
use transactions_integrity::*;

mod authority;
mod resolution;
pub use authority::{
    AppliedTransactionConflictResolution, TransactionConflictApprovalOutput,
    TransactionConflictResolutionOutput,
};
pub use resolution::{
    TransactionResolution, TransactionResolutionReason, TransactionResolutionState,
    TRANSACTION_CONFLICT_POLICY_VERSION,
};

/// Create a new transaction (buyer initiates purchase)
///
/// This starts the transaction lifecycle. The buyer creates the transaction
/// in Pending state, and the seller must confirm it.
#[hdk_extern]
pub fn create_transaction(input: CreateTransactionInput) -> ExternResult<TransactionOutput> {
    let agent_info = agent_info()?;
    let buyer = agent_info.agent_initial_pubkey.clone();

    // The client supplies seller and total for backwards-compatible wire
    // compatibility, but neither value is trusted. Resolve the listing from
    // the listings zome and require the submitted terms to match the DHT.
    let listing = get_listing_for_purchase(input.listing_hash.clone())?;
    let expected_total = validate_purchase_terms(&input, &listing, &buyer)
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))?;

    // Create transaction entry from verified listing terms.
    let transaction = Transaction {
        buyer: buyer.clone(),
        seller: listing.seller_agent_id.clone(),
        listing_hash: listing.listing_hash.clone(),
        quantity: input.quantity,
        total_price_cents: expected_total,
        status: TransactionStatus::Pending,
        created_at: time::now()?,
        updated_at: time::now()?,
        tracking_info: None,
        epistemic: EpistemicClassification {
            // Transaction starts as testimonial (E1)
            empirical: EmpiricalLevel::E1Testimonial,
            // Communal agreement between buyer-seller (N1)
            normative: NormativeLevel::N1Communal,
            // Temporal during transaction (M1)
            materiality: MaterialityLevel::M1Temporal,
        },
    };

    let action_hash = create_entry(&EntryTypes::Transaction(transaction.clone()))?;

    // Create links for discovery
    create_link(
        transaction.buyer.clone(),
        action_hash.clone(),
        LinkTypes::BuyerToTransactions,
        (),
    )?;

    create_link(
        transaction.seller.clone(),
        action_hash.clone(),
        LinkTypes::SellerToTransactions,
        (),
    )?;

    create_link(
        transaction.listing_hash.clone(),
        action_hash.clone(),
        LinkTypes::ListingToTransactions,
        (),
    )?;

    // Emit monitoring metric
    monitoring::emit_metric(
        monitoring::MetricType::TransactionCreated,
        transaction.total_price_cents as f64,
        Some(transaction.buyer.clone()),
        Some(format!(
            "seller:{:?},quantity:{}",
            transaction.seller, transaction.quantity
        )),
    )?;

    Ok(TransactionOutput {
        transaction_hash: action_hash,
        transaction,
    })
}

/// Get the single current transaction revision for an arbitrary Create/Update hash.
///
/// This follows the locally observed update tree. Concurrent heads are returned as
/// an error rather than choosing a silent winner; callers that need conflict evidence
/// should use `get_transaction_resolution`.
#[hdk_extern]
pub fn get_transaction(transaction_hash: ActionHash) -> ExternResult<Option<TransactionOutput>> {
    let Some(resolution) = resolution::resolve_transaction(transaction_hash)? else {
        return Ok(None);
    };
    resolution
        .into_resolved()
        .map(Some)
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))
}

/// Resolve the complete locally observed update tree for a transaction.
///
/// A single leaf is returned as `canonical`; concurrent leaves are returned as
/// an explicit conflict and must not be mutated until the application resolves it.
#[hdk_extern]
pub fn get_transaction_resolution(
    transaction_hash: ActionHash,
) -> ExternResult<Option<TransactionResolution>> {
    resolution::resolve_transaction(transaction_hash)
}


/// Approve one existing branch of an unsafe transaction conflict.
#[hdk_extern]
pub fn approve_transaction_conflict(
    input: ApproveTransactionConflictInput,
) -> ExternResult<TransactionConflictApprovalOutput> {
    let resolution = resolution::resolve_transaction(input.transaction_hash)?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Transaction not found".into()))
    })?;
    authority::ensure_resolution_policy_version(resolution.policy_version)?;
    if resolution.state != TransactionResolutionState::Conflicted {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Conflict approval requires an unresolved multi-head transaction".into(),
        )));
    }
    let selected = authority::selected_head(&resolution.heads, &input.selected_head_hash)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Selected head is not a current transaction leaf".into(),
            ))
        })?;
    let caller = agent_info()?.agent_initial_pubkey;
    if caller != selected.transaction.buyer && caller != selected.transaction.seller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the authoritative buyer or seller may approve a conflict branch".into(),
        )));
    }

    authority::create_conflict_approval(
        resolution.root_transaction_hash,
        authority::current_head_hashes(&resolution.heads),
        input.selected_head_hash,
        input.rationale,
    )
}

/// Publish the final bilateral decision after buyer and seller approve the same branch.
#[hdk_extern]
pub fn finalize_bilateral_transaction_conflict(
    input: FinalizeBilateralTransactionConflictInput,
) -> ExternResult<TransactionResolution> {
    let current = resolution::resolve_transaction(input.transaction_hash)?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Transaction not found".into()))
    })?;
    authority::ensure_resolution_policy_version(current.policy_version)?;
    if current.state != TransactionResolutionState::Conflicted {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Bilateral finalization requires an unresolved multi-head transaction".into(),
        )));
    }

    let buyer_approval = authority::get_conflict_approval(input.buyer_approval_hash.clone())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest("Buyer approval not found".into()))
        })?;
    let seller_approval = authority::get_conflict_approval(input.seller_approval_hash.clone())?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest("Seller approval not found".into()))
        })?;
    let expected_heads = authority::current_head_hashes(&current.heads);
    if buyer_approval.approval.head_hashes != expected_heads
        || seller_approval.approval.head_hashes != expected_heads
        || buyer_approval.approval.selected_head_hash
            != seller_approval.approval.selected_head_hash
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Buyer and seller approvals do not bind the same current conflict and selected head"
                .into(),
        )));
    }

    authority::create_conflict_resolution(
        current.root_transaction_hash.clone(),
        expected_heads,
        buyer_approval.approval.selected_head_hash,
        TransactionConflictAuthority::Bilateral {
            buyer_approval_hash: input.buyer_approval_hash,
            seller_approval_hash: input.seller_approval_hash,
        },
        input.summary,
    )?;

    resolution::resolve_transaction(current.root_transaction_hash)?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Transaction disappeared".into())))
}


/// Publish an arbitration-authorized projection for an exact unsafe conflict.
#[hdk_extern]
pub fn apply_arbitration_transaction_conflict(
    input: ApplyArbitrationTransactionConflictInput,
) -> ExternResult<TransactionResolution> {
    let current = resolution::resolve_transaction(input.transaction_hash)?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Transaction not found".into()))
    })?;
    authority::ensure_resolution_policy_version(current.policy_version)?;
    if current.state != TransactionResolutionState::Conflicted {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Arbitration projection requires an unresolved multi-head transaction".into(),
        )));
    }
    let expected_heads = authority::current_head_hashes(&current.heads);

    let dispute_resolution: Option<DisputeResolutionAuthorityWire> = remote_calls::call_zome(
        "arbitration",
        "get_dispute_resolution",
        input.dispute_hash.clone(),
    )?;
    let dispute_resolution = dispute_resolution.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Conflict dispute not found".into()))
    })?;
    let dispute = dispute_resolution.current()?;
    if dispute_resolution.root_dispute_hash != input.dispute_hash
        || dispute.dispute.transaction_hash != current.root_transaction_hash
        || dispute.dispute.conflict_heads != expected_heads
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Arbitration dispute is not bound to this exact transaction conflict".into(),
        )));
    }
    if !matches!(
        dispute.dispute.status,
        DisputeStatusAuthorityWire::ResolvedBuyer | DisputeStatusAuthorityWire::ResolvedSeller
    ) || dispute.dispute.result_hash.as_ref() != Some(&input.result_hash)
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Conflict dispute is not resolved by the supplied arbitration result".into(),
        )));
    }
    let caller = agent_info()?.agent_initial_pubkey;
    if !dispute.dispute.arbitrators.contains(&caller) {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only an assigned arbitrator may publish the transaction projection".into(),
        )));
    }

    let result: Option<ArbitrationResultOutputAuthorityWire> = remote_calls::call_zome(
        "arbitration",
        "get_arbitration_result",
        input.dispute_hash.clone(),
    )?;
    let result = result.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Arbitration result not found".into()))
    })?;
    if result.result_hash != input.result_hash
        || result.result.dispute_hash != input.dispute_hash
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Supplied result does not match the dispute's unique result".into(),
        )));
    }

    let mut winner_heads = Vec::new();
    for head in &current.heads {
        let record = get(head.transaction_hash.clone(), GetOptions::default())?.ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "A transaction conflict head became unavailable".into(),
            ))
        })?;
        if record.action().author() == &result.result.winner {
            winner_heads.push(head.transaction_hash.clone());
        }
    }
    if winner_heads.len() != 1 {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Arbitration winner authored {} conflict heads; exactly one is required",
            winner_heads.len()
        ))));
    }
    let selected_head_hash = winner_heads.remove(0);

    authority::create_conflict_resolution(
        current.root_transaction_hash.clone(),
        expected_heads,
        selected_head_hash,
        TransactionConflictAuthority::Arbitration {
            dispute_hash: input.dispute_hash,
            result_hash: input.result_hash,
        },
        input.summary,
    )?;

    resolution::resolve_transaction(current.root_transaction_hash)?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Transaction disappeared".into())))
}

#[hdk_extern]
pub fn get_transaction_conflict_approvals(
    transaction_hash: ActionHash,
) -> ExternResult<Vec<TransactionConflictApprovalOutput>> {
    let current = resolution::resolve_transaction(transaction_hash)?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Transaction not found".into()))
    })?;
    authority::get_conflict_approvals(current.root_transaction_hash)
}

#[hdk_extern]
pub fn get_transaction_conflict_resolutions(
    transaction_hash: ActionHash,
) -> ExternResult<Vec<TransactionConflictResolutionOutput>> {
    let current = resolution::resolve_transaction(transaction_hash)?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Transaction not found".into()))
    })?;
    authority::get_conflict_resolutions(current.root_transaction_hash)
}

/// Get all transactions for the current user (as buyer or seller)
#[hdk_extern]
pub fn get_my_transactions(_: ()) -> ExternResult<TransactionsResponse> {
    let agent_info = agent_info()?;
    let agent = agent_info.agent_initial_pubkey;

    let mut transactions = Vec::new();

    // Get transactions as buyer
    // Use shared utility for get_links
    let buyer_links = link_queries::get_links_local(agent.clone(), LinkTypes::BuyerToTransactions)?;

    for link in buyer_links {
        if let Some(action_hash) = link.target.into_action_hash() {
            if let Some(output) = get_transaction(action_hash)? {
                transactions.push(output);
            }
        }
    }

    // Get transactions as seller
    // Use shared utility for get_links
    let seller_links = link_queries::get_links_local(agent, LinkTypes::SellerToTransactions)?;

    for link in seller_links {
        if let Some(action_hash) = link.target.into_action_hash() {
            if let Some(output) = get_transaction(action_hash)? {
                // Avoid duplicates (in case same agent is buyer and seller)
                if !transactions
                    .iter()
                    .any(|t| t.transaction_hash == output.transaction_hash)
                {
                    transactions.push(output);
                }
            }
        }
    }

    Ok(TransactionsResponse { transactions })
}

/// Get conflict-aware transaction resolutions for the current user.
///
/// Unlike `get_my_transactions`, this endpoint preserves branch evidence so the UI
/// can stop lifecycle actions and present the competing heads.
#[hdk_extern]
pub fn get_my_transaction_resolutions(_: ()) -> ExternResult<TransactionResolutionsResponse> {
    let agent = agent_info()?.agent_initial_pubkey;
    let mut roots = Vec::new();

    for link_type in [LinkTypes::BuyerToTransactions, LinkTypes::SellerToTransactions] {
        for link in link_queries::get_links_local(agent.clone(), link_type)? {
            let Some(action_hash) = link.target.into_action_hash() else {
                continue;
            };
            let Some(resolution) = resolution::resolve_transaction(action_hash)? else {
                continue;
            };
            if roots.contains(&resolution.root_transaction_hash) {
                continue;
            }
            roots.push(resolution.root_transaction_hash.clone());
        }
    }

    roots.sort_by_key(ToString::to_string);
    let mut resolutions = Vec::with_capacity(roots.len());
    for root in roots {
        if let Some(resolution) = resolution::resolve_transaction(root)? {
            resolutions.push(resolution);
        }
    }

    Ok(TransactionResolutionsResponse { resolutions })
}

/// Seller confirms the transaction
///
/// State transition: Pending → Confirmed
#[hdk_extern]
pub fn confirm_transaction(transaction_hash: ActionHash) -> ExternResult<TransactionOutput> {
    update_transaction_status(
        transaction_hash,
        TransactionStatus::Confirmed,
        None,
        vec![TransactionStatus::Pending],
        RequiredParty::Seller,
    )
}

/// Seller marks transaction as shipped
///
/// State transition: Confirmed → Shipped
#[hdk_extern]
pub fn mark_shipped(input: MarkShippedInput) -> ExternResult<TransactionOutput> {
    update_transaction_status(
        input.transaction_hash,
        TransactionStatus::Shipped,
        input.tracking_info,
        vec![TransactionStatus::Confirmed],
        RequiredParty::Seller,
    )
}

/// Buyer confirms delivery and projects one immutable fulfillment event.
///
/// State transition: Shipped → Delivered. Retrying against an already Delivered
/// transaction does not create a new revision; it only recovers the idempotent
/// reputation projection.
#[hdk_extern]
pub fn confirm_delivery(transaction_hash: ActionHash) -> ExternResult<TransactionOutput> {
    let (_root, current) = require_resolved_transaction(transaction_hash.clone())?;
    let caller = agent_info()?.agent_initial_pubkey;
    if caller != current.transaction.buyer {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the buyer may confirm delivery".into()
        )));
    }

    let delivered = if current.transaction.status == TransactionStatus::Delivered {
        current
    } else {
        update_transaction_status(
            transaction_hash,
            TransactionStatus::Delivered,
            None,
            vec![TransactionStatus::Shipped],
            RequiredParty::Buyer,
        )?
    };
    project_fulfillment_reputation(delivered.transaction_hash.clone())?;
    Ok(delivered)
}

fn project_fulfillment_reputation(transaction_hash: ActionHash) -> ExternResult<()> {
    let _: ReputationEventOutputWire = remote_calls::call_zome(
        "reputation",
        "record_fulfillment_reputation",
        transaction_hash,
    )?;
    Ok(())
}

/// Compatibility wrapper for the former local `Delivered -> Completed` update.
///
/// Settlement is now represented by an authoritative Finance record instead of
/// a Marketplace transaction status that a modified local client could forge.
/// The returned transaction therefore remains `Delivered`; callers must inspect
/// `get_transaction_settlement_status` for economic finality.
#[hdk_extern]
pub fn complete_transaction(transaction_hash: ActionHash) -> ExternResult<TransactionOutput> {
    let settlement = settle_transaction(transaction_hash.clone())?;
    if !settlement.settled {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Finance settlement did not reach Completed: {}",
            settlement
                .error
                .unwrap_or_else(|| format!("state {:?}", settlement.state))
        ))));
    }
    let (_root, current) = require_resolved_transaction(transaction_hash)?;
    Ok(current)
}

/// Initiate or recover one idempotent Finance settlement for a delivered
/// transaction. This function never writes a Marketplace `Completed` revision.
#[hdk_extern]
pub fn settle_transaction(
    transaction_hash: ActionHash,
) -> ExternResult<TransactionSettlementResult> {
    let (root_transaction_hash, current) = require_resolved_transaction(transaction_hash)?;
    let caller = agent_info()?.agent_initial_pubkey;
    if caller != current.transaction.buyer {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the buyer may initiate settlement".into()
        )));
    }
    if current.transaction.status != TransactionStatus::Delivered {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Settlement requires Delivered status, found {:?}",
            current.transaction.status
        ))));
    }
    process_transaction_settlement(root_transaction_hash, current)
}

/// Recover Finance settlement state by the stable transaction root. Buyer and
/// seller may query; Finance independently authenticates the caller as a party.
#[hdk_extern]
pub fn get_transaction_settlement_status(
    transaction_hash: ActionHash,
) -> ExternResult<TransactionSettlementResult> {
    let (root_transaction_hash, current) = require_resolved_transaction(transaction_hash)?;
    let caller = agent_info()?.agent_initial_pubkey;
    if caller != current.transaction.buyer && caller != current.transaction.seller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only transaction parties may inspect settlement status".into()
        )));
    }
    query_transaction_settlement(root_transaction_hash, current)
}

/// Open one recoverable arbitration case for a transaction.
///
/// The transaction revision is moved to Disputed first, then arbitration is
/// called idempotently. If case creation or assignment fails, retrying this
/// function reuses the Disputed revision and the existing case rather than
/// creating another transaction branch.
#[hdk_extern]
pub fn open_dispute(input: OpenDisputeInput) -> ExternResult<OpenDisputeOutput> {
    validate_dispute_request(&input.reason, &input.evidence_cids)
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))?;

    let (root_transaction_hash, current) =
        require_resolved_transaction(input.transaction_hash.clone())?;
    let caller = agent_info()?.agent_initial_pubkey;

    if caller != current.transaction.buyer && caller != current.transaction.seller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only buyer or seller can dispute transaction".into()
        )));
    }
    if matches!(
        current.transaction.status,
        TransactionStatus::Completed | TransactionStatus::Cancelled
    ) {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Cannot dispute transaction with status {:?}",
            current.transaction.status
        ))));
    }

    let disputed = if current.transaction.status == TransactionStatus::Disputed {
        current
    } else {
        let mut updated_transaction = current.transaction.clone();
        updated_transaction.status = TransactionStatus::Disputed;
        updated_transaction.updated_at = time::now()?;
        let transaction_hash = update_entry(current.transaction_hash, &updated_transaction)?;
        TransactionOutput {
            transaction_hash,
            transaction: updated_transaction,
        }
    };

    let dispute: DisputeOutputWire = remote_calls::call_zome(
        "arbitration",
        "file_dispute",
        FileDisputeInputWire {
            transaction_hash: root_transaction_hash,
            reason: input.reason,
            evidence_cids: input.evidence_cids,
        },
    )?;
    let dispute_resolution: Option<DisputeResolutionRootWire> = remote_calls::call_zome(
        "arbitration",
        "get_dispute_resolution",
        dispute.dispute_hash,
    )?;
    let root_dispute_hash = dispute_resolution
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Arbitration case was created but its stable root could not be resolved".into(),
            ))
        })?
        .root_dispute_hash;

    monitoring::emit_metric(
        monitoring::MetricType::TransactionDisputed,
        disputed.transaction.total_price_cents as f64,
        Some(caller),
        Some(format!(
            "buyer:{:?},seller:{:?},dispute:{}",
            disputed.transaction.buyer, disputed.transaction.seller, root_dispute_hash
        )),
    )?;

    Ok(OpenDisputeOutput {
        transaction: disputed,
        dispute_hash: root_dispute_hash,
    })
}

/// Backwards-compatible wrapper for callers that do not attach evidence CIDs.
#[hdk_extern]
pub fn dispute_transaction(input: DisputeTransactionInput) -> ExternResult<TransactionOutput> {
    Ok(open_dispute(OpenDisputeInput {
        transaction_hash: input.transaction_hash,
        reason: input.reason,
        evidence_cids: Vec::new(),
    })?
    .transaction)
}

/// Cancel a transaction
///
/// State transition: Pending/Confirmed → Cancelled
#[hdk_extern]
pub fn cancel_transaction(transaction_hash: ActionHash) -> ExternResult<TransactionOutput> {
    // Resolve the canonical head before applying cancellation.
    let (_root_transaction_hash, current) = require_resolved_transaction(transaction_hash)?;

    let agent_info = agent_info()?;
    let caller = agent_info.agent_initial_pubkey;

    // Verify caller is buyer or seller
    if caller != current.transaction.buyer && caller != current.transaction.seller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only buyer or seller can cancel transaction".into()
        )));
    }

    // Can only cancel from Pending or Confirmed states
    match current.transaction.status {
        TransactionStatus::Pending | TransactionStatus::Confirmed => {}
        _ => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Cannot cancel transaction from status {:?}",
                current.transaction.status
            ))));
        }
    }

    // Update transaction status
    let mut updated_transaction = current.transaction.clone();
    updated_transaction.status = TransactionStatus::Cancelled;
    updated_transaction.updated_at = time::now()?;

    let new_action_hash = update_entry(current.transaction_hash, &updated_transaction)?;

    Ok(TransactionOutput {
        transaction_hash: new_action_hash,
        transaction: updated_transaction,
    })
}

/// Get transactions for a specific listing
#[hdk_extern]
pub fn get_listing_transactions(listing_hash: ActionHash) -> ExternResult<TransactionsResponse> {
    // Use shared utility for get_links
    let links = link_queries::get_links_local(listing_hash, LinkTypes::ListingToTransactions)?;

    let mut transactions = Vec::new();

    for link in links {
        if let Some(action_hash) = link.target.into_action_hash() {
            if let Some(output) = get_transaction(action_hash)? {
                transactions.push(output);
            }
        }
    }

    Ok(TransactionsResponse { transactions })
}


#[derive(Serialize, Deserialize, Debug, Clone)]
struct ListingOutputForPurchase {
    listing_hash: ActionHash,
    listing: Listing,
    seller_agent_id: AgentPubKey,
}

fn get_listing_for_purchase(listing_hash: ActionHash) -> ExternResult<ListingOutputForPurchase> {
    let response = call(
        CallTargetCell::Local,
        ZomeName::from("listings"),
        FunctionName::from("get_listing"),
        None,
        listing_hash,
    )?;

    match response {
        ZomeCallResponse::Ok(extern_io) => {
            let listing: Option<ListingOutputForPurchase> = extern_io.decode().map_err(|error| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "Failed to decode listings.get_listing response: {error:?}"
                )))
            })?;
            listing.ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest("Listing not found".into()))
            })
        }
        other => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "listings.get_listing failed: {other:?}"
        )))),
    }
}

fn validate_purchase_terms(
    input: &CreateTransactionInput,
    listing: &ListingOutputForPurchase,
    buyer: &AgentPubKey,
) -> Result<u64, String> {
    security::validate_quantity(input.quantity)?;

    if input.listing_hash != listing.listing_hash {
        return Err("Listing response did not match the requested listing".into());
    }
    if listing.listing.status != ListingStatus::Active {
        return Err("Listing is not active".into());
    }
    if input.quantity > listing.listing.quantity_available {
        return Err(format!(
            "Requested quantity {} exceeds available quantity {}",
            input.quantity, listing.listing.quantity_available
        ));
    }
    if buyer == &listing.seller_agent_id {
        return Err("Seller cannot purchase their own listing".into());
    }
    if input.seller != listing.seller_agent_id {
        return Err("Submitted seller does not match the listing owner".into());
    }

    let expected_total = listing
        .listing
        .price_cents
        .checked_mul(u64::from(input.quantity))
        .ok_or_else(|| "Transaction total overflow".to_string())?;

    if input.total_price_cents != expected_total {
        return Err(format!(
            "Submitted total {} does not match listing total {}",
            input.total_price_cents, expected_total
        ));
    }

    Ok(expected_total)
}

fn require_resolved_transaction(
    transaction_hash: ActionHash,
) -> ExternResult<(ActionHash, TransactionOutput)> {
    let resolution = resolution::resolve_transaction(transaction_hash)?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest("Transaction not found".into()))
    })?;
    let root = resolution.root_transaction_hash.clone();
    let current = resolution
        .into_resolved()
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))?;
    Ok((root, current))
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
enum DisputeResolutionStateAuthorityWire {
    Resolved,
    Conflicted,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct DisputeResolutionAuthorityWire {
    root_dispute_hash: ActionHash,
    state: DisputeResolutionStateAuthorityWire,
    canonical: Option<DisputeOutputAuthorityWire>,
    heads: Vec<DisputeOutputAuthorityWire>,
}

impl DisputeResolutionAuthorityWire {
    fn current(&self) -> ExternResult<&DisputeOutputAuthorityWire> {
        match (&self.state, &self.canonical) {
            (DisputeResolutionStateAuthorityWire::Resolved, Some(current)) => Ok(current),
            (DisputeResolutionStateAuthorityWire::Resolved, None) => Err(wasm_error!(
                WasmErrorInner::Guest("Resolved dispute omitted its current revision".into())
            )),
            (DisputeResolutionStateAuthorityWire::Conflicted, _) => Err(wasm_error!(
                WasmErrorInner::Guest(format!(
                    "Dispute has {} concurrent heads",
                    self.heads.len()
                ))
            )),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct DisputeOutputAuthorityWire {
    dispute_hash: ActionHash,
    dispute: DisputeAuthorityWire,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct DisputeAuthorityWire {
    transaction_hash: ActionHash,
    buyer: AgentPubKey,
    seller: AgentPubKey,
    conflict_heads: Vec<ActionHash>,
    status: DisputeStatusAuthorityWire,
    arbitrators: Vec<AgentPubKey>,
    result_hash: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
enum DisputeStatusAuthorityWire {
    Filed,
    UnderReview,
    Voting,
    ResolvedBuyer,
    ResolvedSeller,
    Withdrawn,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ArbitrationResultOutputAuthorityWire {
    result_hash: ActionHash,
    result: ArbitrationResultAuthorityWire,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ArbitrationResultAuthorityWire {
    dispute_hash: ActionHash,
    winner: AgentPubKey,
    loser: AgentPubKey,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct FileDisputeInputWire {
    transaction_hash: ActionHash,
    reason: String,
    evidence_cids: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ReputationEventOutputWire {
    event_hash: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct DisputeOutputWire {
    dispute_hash: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct DisputeResolutionRootWire {
    root_dispute_hash: ActionHash,
}

fn validate_dispute_request(reason: &str, evidence_cids: &[String]) -> Result<(), String> {
    if reason.trim().is_empty() || reason.len() > 5000 {
        return Err("Dispute reason must be 1-5000 characters".into());
    }
    if evidence_cids.len() > 32 {
        return Err("Disputes may attach at most 32 evidence CIDs".into());
    }
    for cid in evidence_cids {
        let valid = (cid.starts_with("Qm") && cid.len() == 46)
            || (cid.starts_with('b') && (50..=100).contains(&cid.len()));
        if !valid {
            return Err(format!("Invalid IPFS CID: {cid}"));
        }
    }
    Ok(())
}

// ===== Finance Settlement Bridge =====

const FINANCE_ROLE: &str = "finance";
const FINANCE_ZOME: &str = "finance_bridge";
const MARKETPLACE_HAPP_ID: &str = "mycelix-marketplace";

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum TransactionSettlementState {
    Unavailable,
    NotFound,
    Pending,
    Processing,
    Completed,
    Failed,
    Refunded,
    Disputed,
    Invalid,
}

/// Authoritative settlement projection for one stable Marketplace transaction.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TransactionSettlementResult {
    pub root_transaction_hash: ActionHash,
    pub transaction_revision_hash: ActionHash,
    pub state: TransactionSettlementState,
    pub settled: bool,
    pub idempotency_reference: String,
    pub finance_payment_id: Option<String>,
    pub finance_action_hash: Option<ActionHash>,
    pub error: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
enum FinancePaymentStatusWire {
    Pending,
    Processing,
    Completed,
    Failed,
    Refunded,
    Disputed,
}

#[derive(Serialize, Deserialize, Debug, Clone, SerializedBytes)]
struct FinancePaymentWire {
    id: String,
    source_happ: String,
    from_did: String,
    to_did: String,
    amount: u64,
    currency: String,
    reference: String,
    status: FinancePaymentStatusWire,
    created_at: Timestamp,
    completed_at: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct FinancePaymentInputWire {
    source_happ: String,
    from_did: String,
    to_did: String,
    amount: u64,
    currency: String,
    reference: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct FinancePaymentLookupWire {
    source_happ: String,
    reference: String,
}

fn settlement_reference(root_transaction_hash: &ActionHash) -> String {
    format!("marketplace_tx:{root_transaction_hash}")
}

fn settlement_terms(
    root_transaction_hash: &ActionHash,
    transaction: &Transaction,
) -> FinancePaymentInputWire {
    FinancePaymentInputWire {
        source_happ: MARKETPLACE_HAPP_ID.into(),
        from_did: format!("did:mycelix:{}", transaction.buyer),
        to_did: format!("did:mycelix:{}", transaction.seller),
        amount: transaction.total_price_cents,
        currency: "SAP".into(),
        reference: settlement_reference(root_transaction_hash),
    }
}

fn decode_finance_payment(record: &Record) -> ExternResult<FinancePaymentWire> {
    record
        .entry()
        .to_app_option::<FinancePaymentWire>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode Finance payment: {e:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Finance response did not contain a payment entry".into()
            ))
        })
}

fn validate_finance_payment(
    expected: &FinancePaymentInputWire,
    payment: &FinancePaymentWire,
) -> Result<(), String> {
    if payment.source_happ != expected.source_happ
        || payment.from_did != expected.from_did
        || payment.to_did != expected.to_did
        || payment.amount != expected.amount
        || payment.currency != expected.currency
        || payment.reference != expected.reference
    {
        return Err("Finance payment does not match authoritative transaction terms".into());
    }
    if payment.status == FinancePaymentStatusWire::Completed && payment.completed_at.is_none() {
        return Err("Finance payment claims Completed without a completion timestamp".into());
    }
    Ok(())
}

fn project_finance_record(
    root_transaction_hash: ActionHash,
    current: TransactionOutput,
    record: Record,
) -> TransactionSettlementResult {
    let reference = settlement_reference(&root_transaction_hash);
    let action_hash = record.action_address().clone();
    match decode_finance_payment(&record).and_then(|payment| {
        let expected = settlement_terms(&root_transaction_hash, &current.transaction);
        validate_finance_payment(&expected, &payment)
            .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))?;
        Ok(payment)
    }) {
        Ok(payment) => {
            let state = match payment.status {
                FinancePaymentStatusWire::Pending => TransactionSettlementState::Pending,
                FinancePaymentStatusWire::Processing => TransactionSettlementState::Processing,
                FinancePaymentStatusWire::Completed => TransactionSettlementState::Completed,
                FinancePaymentStatusWire::Failed => TransactionSettlementState::Failed,
                FinancePaymentStatusWire::Refunded => TransactionSettlementState::Refunded,
                FinancePaymentStatusWire::Disputed => TransactionSettlementState::Disputed,
            };
            TransactionSettlementResult {
                root_transaction_hash,
                transaction_revision_hash: current.transaction_hash,
                settled: state == TransactionSettlementState::Completed,
                state,
                idempotency_reference: reference,
                finance_payment_id: Some(payment.id),
                finance_action_hash: Some(action_hash),
                error: None,
            }
        }
        Err(error) => TransactionSettlementResult {
            root_transaction_hash,
            transaction_revision_hash: current.transaction_hash,
            state: TransactionSettlementState::Invalid,
            settled: false,
            idempotency_reference: reference,
            finance_payment_id: None,
            finance_action_hash: Some(action_hash),
            error: Some(format!("{error:?}")),
        },
    }
}

fn unavailable_settlement(
    root_transaction_hash: ActionHash,
    current: TransactionOutput,
    error: impl Into<String>,
) -> TransactionSettlementResult {
    TransactionSettlementResult {
        idempotency_reference: settlement_reference(&root_transaction_hash),
        root_transaction_hash,
        transaction_revision_hash: current.transaction_hash,
        state: TransactionSettlementState::Unavailable,
        settled: false,
        finance_payment_id: None,
        finance_action_hash: None,
        error: Some(error.into()),
    }
}

fn call_finance_record(
    function: &str,
    payload: impl Serialize + std::fmt::Debug,
) -> Result<Record, String> {
    let encoded = ExternIO::encode(payload).map_err(|e| e.to_string())?;
    match call(
        CallTargetCell::OtherRole(FINANCE_ROLE.into()),
        ZomeName::from(FINANCE_ZOME),
        FunctionName::from(function),
        None,
        encoded,
    ) {
        Ok(ZomeCallResponse::Ok(data)) => data
            .decode::<Record>()
            .map_err(|e| format!("Finance returned an invalid Record: {e:?}")),
        Ok(other) => Err(format!("Finance rejected the request: {other:?}")),
        Err(error) => Err(format!("Finance role unavailable: {error:?}")),
    }
}

fn call_finance_optional_record(
    function: &str,
    payload: impl Serialize + std::fmt::Debug,
) -> Result<Option<Record>, String> {
    let encoded = ExternIO::encode(payload).map_err(|e| e.to_string())?;
    match call(
        CallTargetCell::OtherRole(FINANCE_ROLE.into()),
        ZomeName::from(FINANCE_ZOME),
        FunctionName::from(function),
        None,
        encoded,
    ) {
        Ok(ZomeCallResponse::Ok(data)) => data
            .decode::<Option<Record>>()
            .map_err(|e| format!("Finance returned an invalid optional Record: {e:?}")),
        Ok(other) => Err(format!("Finance rejected the request: {other:?}")),
        Err(error) => Err(format!("Finance role unavailable: {error:?}")),
    }
}

fn process_transaction_settlement(
    root_transaction_hash: ActionHash,
    current: TransactionOutput,
) -> ExternResult<TransactionSettlementResult> {
    let expected = settlement_terms(&root_transaction_hash, &current.transaction);
    match call_finance_record("process_payment_remote", expected) {
        Ok(record) => Ok(project_finance_record(root_transaction_hash, current, record)),
        Err(error) => Ok(unavailable_settlement(root_transaction_hash, current, error)),
    }
}

fn query_transaction_settlement(
    root_transaction_hash: ActionHash,
    current: TransactionOutput,
) -> ExternResult<TransactionSettlementResult> {
    let reference = settlement_reference(&root_transaction_hash);
    let lookup = FinancePaymentLookupWire {
        source_happ: MARKETPLACE_HAPP_ID.into(),
        reference: reference.clone(),
    };
    match call_finance_optional_record("verify_payment_status_remote", lookup) {
        Ok(Some(record)) => Ok(project_finance_record(root_transaction_hash, current, record)),
        Ok(None) => Ok(TransactionSettlementResult {
            root_transaction_hash,
            transaction_revision_hash: current.transaction_hash,
            state: TransactionSettlementState::NotFound,
            settled: false,
            idempotency_reference: reference,
            finance_payment_id: None,
            finance_action_hash: None,
            error: None,
        }),
        Err(error) => Ok(unavailable_settlement(root_transaction_hash, current, error)),
    }
}

// ===== Helper Functions =====

/// Update transaction status with validation
/// Which party in the transaction is required to be the caller for a
/// given state transition.
enum RequiredParty {
    Buyer,
    Seller,
}

fn update_transaction_status(
    transaction_hash: ActionHash,
    new_status: TransactionStatus,
    tracking_info: Option<String>,
    allowed_from_states: Vec<TransactionStatus>,
    required_caller: RequiredParty,
) -> ExternResult<TransactionOutput> {
    // Always extend the single observed leaf, even if the caller supplied the
    // stable root or an older revision hash. Conflicts fail closed.
    let (_root_transaction_hash, current) = require_resolved_transaction(transaction_hash)?;

    // Verify caller is the party authorized for this transition — mirrors
    // the buyer/seller check already used correctly by dispute_transaction
    // and cancel_transaction.
    let caller = agent_info()?.agent_initial_pubkey;
    let expected = match required_caller {
        RequiredParty::Buyer => &current.transaction.buyer,
        RequiredParty::Seller => &current.transaction.seller,
    };
    if caller != *expected {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Only the {} may perform this transition",
            match required_caller {
                RequiredParty::Buyer => "buyer",
                RequiredParty::Seller => "seller",
            }
        ))));
    }

    // Verify state transition is valid
    if !allowed_from_states.contains(&current.transaction.status) {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid state transition from {:?} to {:?}",
            current.transaction.status, new_status
        ))));
    }

    // Create updated transaction
    let mut updated_transaction = current.transaction.clone();
    updated_transaction.status = new_status;
    updated_transaction.updated_at = time::now()?;

    if let Some(info) = tracking_info {
        updated_transaction.tracking_info = Some(info);
    }

    // Update entry
    let new_action_hash = update_entry(current.transaction_hash, &updated_transaction)?;

    Ok(TransactionOutput {
        transaction_hash: new_action_hash,
        transaction: updated_transaction,
    })
}

// ===== Input/Output Types =====

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApproveTransactionConflictInput {
    pub transaction_hash: ActionHash,
    pub selected_head_hash: ActionHash,
    pub rationale: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FinalizeBilateralTransactionConflictInput {
    pub transaction_hash: ActionHash,
    pub buyer_approval_hash: ActionHash,
    pub seller_approval_hash: ActionHash,
    pub summary: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApplyArbitrationTransactionConflictInput {
    pub transaction_hash: ActionHash,
    pub dispute_hash: ActionHash,
    pub result_hash: ActionHash,
    pub summary: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateTransactionInput {
    pub seller: AgentPubKey,
    pub listing_hash: ActionHash,
    pub quantity: u32,
    pub total_price_cents: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TransactionOutput {
    pub transaction_hash: ActionHash,
    pub transaction: Transaction,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TransactionsResponse {
    pub transactions: Vec<TransactionOutput>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TransactionResolutionsResponse {
    pub resolutions: Vec<TransactionResolution>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MarkShippedInput {
    pub transaction_hash: ActionHash,
    pub tracking_info: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OpenDisputeInput {
    pub transaction_hash: ActionHash,
    pub reason: String,
    pub evidence_cids: Vec<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct OpenDisputeOutput {
    pub transaction: TransactionOutput,
    pub dispute_hash: ActionHash,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DisputeTransactionInput {
    pub transaction_hash: ActionHash,
    pub reason: String,
}

// ===== Tests =====
#[cfg(test)]
mod tests;
