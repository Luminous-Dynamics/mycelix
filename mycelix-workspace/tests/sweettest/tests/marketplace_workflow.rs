// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Marketplace hApp sweettest integration tests.
//!
//! Tests P2P commerce workflows including listings, transactions, reputation,
//! and MATL trust scoring using the Holochain sweettest framework.
//!
//! Prerequisites:
//!   cd mycelix-marketplace/backend && cargo build --release --target wasm32-unknown-unknown
//!   hc dna pack . -o mycelix_marketplace.dna
//!
//! Run: cargo test --release -p mycelix-sweettest -- --ignored marketplace
//!
//! Updated for Holochain 0.6 sweettest API.
//!
//! All zome-call inputs/outputs below use typed "Mirror" structs (not
//! `serde_json::Value`) wherever the real zome struct contains an
//! ActionHash/AgentPubKey/Timestamp field -- Holochain's wire format
//! (`holochain_serialized_bytes`, rmp-serde with `.with_struct_map()`) encodes
//! those HoloHash types as raw bytes, which `serde_json::Value`'s Deserialize
//! impl cannot represent ("invalid type: byte array, expected any valid JSON
//! value"). Struct field ORDER doesn't matter (map-based wire format, not
//! positional) -- only field NAMES and TYPES must match the real zome structs
//! in zomes/{listings,transactions,reputation}/{integrity,coordinator}/src/lib.rs.
//! Plain-data-only inputs (no HoloHash fields, e.g. CreateListingInput) still
//! use `serde_json::json!()` for brevity.

mod harness;

use harness::*;
use holochain::prelude::*;
use serial_test::serial;

// A valid CIDv0 (Qm + 44 base58 chars, matches is_valid_ipfs_cid in
// zomes/listings/integrity/src/lib.rs) -- reused from that zome's own tests.
const VALID_CID: &str = "QmYwAPJzv5CZsnA625s3Xf2nemtYgPpHdWEz79ojWnPbdG";

// =============================================================================
// Listing Management Tests
// =============================================================================

/// Test: Create a listing and retrieve it.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore] // Requires DNA bundle
async fn test_create_and_get_listing() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 1).await;

    let seller = &agents[0];

    // Create a listing
    let listing_input = serde_json::json!({
        "title": "Handmade Pottery Bowl",
        "description": "Beautiful ceramic bowl, locally crafted",
        "price_cents": 4500,
        "category": "Other",
        "photos_ipfs_cids": [VALID_CID],
        "quantity_available": 1
    });

    let listing_output: ListingOutputMirror = seller
        .call_zome_fn("listings", "create_listing", listing_input)
        .await;

    assert!(
        !listing_output.listing_hash.as_ref().is_empty(),
        "Listing should be created"
    );

    // Retrieve the listing
    let retrieved: Option<ListingOutputMirror> = seller
        .call_zome_fn("listings", "get_listing", listing_output.listing_hash)
        .await;

    assert!(retrieved.is_some(), "Listing should be retrievable");
}

/// Test: List all active listings.
///
/// Previously blocked on a real zome bug (fixed): every `create_link` call
/// in `create_listing` (zomes/listings/coordinator/src/lib.rs, the
/// AgentToListings/CategoryToListings/StatusToListings/AllListings links)
/// targeted `entry_hash` (an `EntryHash`, from `hash_entry(&listing)?`), but
/// every consumer (`get_all_listings`, `get_listings_by_seller` /
/// `get_my_listings`, `get_listings_by_category`) does
/// `link.target.into_action_hash()`, which returns `None` for a target
/// that's actually an `EntryHash`. Every link was silently skipped by the
/// `if let Some(action_hash) = ...` pattern, regardless of how long you
/// waited for DHT integration -- confirmed by retrying up to 15x with a 2s
/// wait_for_dht_sync() between attempts and still getting 0. Fixed by
/// changing all four `create_link` calls to target `action_hash` instead.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_list_active_listings() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 1).await;

    let seller = &agents[0];

    // Create multiple listings
    for i in 1..=3 {
        let listing_input = serde_json::json!({
            "title": format!("Test Listing {}", i),
            "description": format!("Description for listing {}", i),
            "price_cents": 1000 * i as u64,
            "category": "Other",
            "photos_ipfs_cids": [VALID_CID],
            "quantity_available": 5
        });

        let _: ListingOutputMirror = seller
            .call_zome_fn("listings", "create_listing", listing_input)
            .await;
    }

    // get_my_listings is the right function to test "list my listings" intent
    // (there is no `list_active_listings` extern) -- it links directly from
    // the seller's own AgentPubKey, a real, always-valid DHT address, unlike
    // the Path-anchor-based get_all_listings. Retrying rather than a single
    // fixed sleep is the correct way to test an eventually-consistent read --
    // but per the doc comment above, this specific query can never succeed
    // today regardless of wait time, due to the EntryHash/ActionHash bug.
    let mut listings = ListingsResponseMirror { listings: vec![] };
    for _ in 0..15 {
        listings = seller.call_zome_fn("listings", "get_my_listings", ()).await;
        if listings.listings.len() >= 3 {
            break;
        }
        wait_for_dht_sync().await;
    }

    assert!(
        listings.listings.len() >= 3,
        "Should have at least 3 listings, got {}",
        listings.listings.len()
    );
}

/// Test: Update listing price.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_update_listing() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 1).await;

    let seller = &agents[0];

    // Create listing
    let listing_input = serde_json::json!({
        "title": "Update Test Item",
        "description": "Original description",
        "price_cents": 5000,
        "category": "Electronics",
        "photos_ipfs_cids": [VALID_CID],
        "quantity_available": 2
    });

    let listing_output: ListingOutputMirror = seller
        .call_zome_fn("listings", "create_listing", listing_input)
        .await;

    // Update the listing (partial update: only description + price change)
    let update_input = UpdateListingInputMirror {
        listing_hash: listing_output.listing_hash,
        title: None,
        description: Some("Updated description with more details".to_string()),
        price_cents: Some(4500),
        category: None,
        photos_ipfs_cids: None,
        quantity_available: None,
        status: None,
    };

    let updated: ListingOutputMirror = seller
        .call_zome_fn("listings", "update_listing", update_input)
        .await;

    assert!(
        !updated.listing_hash.as_ref().is_empty(),
        "Listing should be updated"
    );
    assert_eq!(
        updated.listing.price_cents, 4500,
        "Listing price should reflect the update"
    );
    assert_eq!(
        updated.listing.description, "Updated description with more details",
        "Listing description should reflect the update"
    );
}

// =============================================================================
// Transaction Lifecycle Tests
// =============================================================================

/// Test: Create transaction between buyer and seller.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_create_transaction() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 2).await;

    let seller = &agents[0];
    let buyer = &agents[1];

    // Seller creates listing
    let listing_input = serde_json::json!({
        "title": "Transaction Test Item",
        "description": "Item for transaction test",
        "price_cents": 10000,
        "category": "Other",
        "photos_ipfs_cids": [VALID_CID],
        "quantity_available": 1
    });

    let listing_output: ListingOutputMirror = seller
        .call_zome_fn("listings", "create_listing", listing_input)
        .await;

    wait_for_dht_sync().await;

    // Buyer initiates transaction
    let transaction_input = CreateTransactionInputMirror {
        seller: seller.agent_pubkey.clone(),
        listing_hash: listing_output.listing_hash,
        quantity: 1,
        total_price_cents: 10000,
    };

    let transaction_output: TransactionOutputMirror = buyer
        .call_zome_fn("transactions", "create_transaction", transaction_input)
        .await;

    assert!(
        !transaction_output.transaction_hash.as_ref().is_empty(),
        "Transaction should be created"
    );
    assert_eq!(transaction_output.transaction.status, "pending");
}

/// Test: Transaction status progression (Pending -> Confirmed -> Shipped).
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_transaction_status_progression() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 2).await;

    let seller = &agents[0];
    let buyer = &agents[1];

    // Create listing and transaction
    let listing_input = serde_json::json!({
        "title": "Status Test Item",
        "description": "Testing status progression",
        "price_cents": 7500,
        "category": "Other",
        "photos_ipfs_cids": [VALID_CID],
        "quantity_available": 1
    });

    let listing_output: ListingOutputMirror = seller
        .call_zome_fn("listings", "create_listing", listing_input)
        .await;

    wait_for_dht_sync().await;

    let transaction_input = CreateTransactionInputMirror {
        seller: seller.agent_pubkey.clone(),
        listing_hash: listing_output.listing_hash,
        quantity: 1,
        total_price_cents: 7500,
    };

    let created: TransactionOutputMirror = buyer
        .call_zome_fn("transactions", "create_transaction", transaction_input)
        .await;

    wait_for_dht_sync().await;

    // Seller confirms the transaction: Pending -> Confirmed. Each mutating
    // call creates a new action (update_entry), so the response's
    // transaction_hash must be threaded into the next call.
    let confirmed: TransactionOutputMirror = seller
        .call_zome_fn(
            "transactions",
            "confirm_transaction",
            created.transaction_hash.clone(),
        )
        .await;
    assert_eq!(confirmed.transaction.status, "confirmed");

    wait_for_dht_sync().await;

    // Seller marks as Shipped: Confirmed -> Shipped
    let ship_input = MarkShippedInputMirror {
        transaction_hash: confirmed.transaction_hash.clone(),
        tracking_info: Some("1Z999AA10123456784".to_string()),
    };

    let shipped: TransactionOutputMirror = seller
        .call_zome_fn("transactions", "mark_shipped", ship_input)
        .await;
    assert_eq!(shipped.transaction.status, "shipped");

    wait_for_dht_sync().await;

    // Get transaction to verify status
    let retrieved: Option<TransactionOutputMirror> = buyer
        .call_zome_fn("transactions", "get_transaction", shipped.transaction_hash)
        .await;

    let retrieved = retrieved.expect("Transaction should be retrievable");
    assert_eq!(retrieved.transaction.status, "shipped");
}

/// Test: Deliver item and complete the shipping leg of a transaction.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_deliver_item() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 2).await;

    let seller = &agents[0];
    let buyer = &agents[1];

    // Quick setup - create listing and transaction
    let listing_input = serde_json::json!({
        "title": "Delivery Test Item",
        "description": "Testing delivery flow",
        "price_cents": 2500,
        "category": "Other",
        "photos_ipfs_cids": [VALID_CID],
        "quantity_available": 1
    });

    let listing_output: ListingOutputMirror = seller
        .call_zome_fn("listings", "create_listing", listing_input)
        .await;

    wait_for_dht_sync().await;

    let transaction_input = CreateTransactionInputMirror {
        seller: seller.agent_pubkey.clone(),
        listing_hash: listing_output.listing_hash,
        quantity: 1,
        total_price_cents: 2500,
    };

    let created: TransactionOutputMirror = buyer
        .call_zome_fn("transactions", "create_transaction", transaction_input)
        .await;

    wait_for_dht_sync().await;

    // Seller confirms, then ships -- confirm_delivery (the real function
    // behind "deliver item") requires the prior state to be Shipped.
    let confirmed: TransactionOutputMirror = seller
        .call_zome_fn(
            "transactions",
            "confirm_transaction",
            created.transaction_hash.clone(),
        )
        .await;

    wait_for_dht_sync().await;

    let shipped: TransactionOutputMirror = seller
        .call_zome_fn(
            "transactions",
            "mark_shipped",
            MarkShippedInputMirror {
                transaction_hash: confirmed.transaction_hash.clone(),
                tracking_info: None,
            },
        )
        .await;

    wait_for_dht_sync().await;

    // Buyer confirms delivery
    let delivered: TransactionOutputMirror = buyer
        .call_zome_fn(
            "transactions",
            "confirm_delivery",
            shipped.transaction_hash.clone(),
        )
        .await;

    assert!(
        !delivered.transaction_hash.as_ref().is_empty(),
        "Delivery should be recorded"
    );
    assert_eq!(delivered.transaction.status, "delivered");
}

// =============================================================================
// Reputation System Tests
// =============================================================================

/// Test: Leave feedback (a review) after a transaction.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_leave_feedback() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 2).await;

    let seller = &agents[0];
    let buyer = &agents[1];

    // Setup a transaction to leave feedback against. submit_review doesn't
    // itself require the transaction to be in any particular status.
    let listing_input = serde_json::json!({
        "title": "Feedback Test Item",
        "description": "Testing feedback system",
        "price_cents": 3000,
        "category": "Other",
        "photos_ipfs_cids": [VALID_CID],
        "quantity_available": 1
    });

    let listing_output: ListingOutputMirror = seller
        .call_zome_fn("listings", "create_listing", listing_input)
        .await;

    wait_for_dht_sync().await;

    let transaction_input = CreateTransactionInputMirror {
        seller: seller.agent_pubkey.clone(),
        listing_hash: listing_output.listing_hash.clone(),
        quantity: 1,
        total_price_cents: 3000,
    };

    let created: TransactionOutputMirror = buyer
        .call_zome_fn("transactions", "create_transaction", transaction_input)
        .await;

    wait_for_dht_sync().await;

    // Buyer leaves a review (feedback) for the seller. The real function is
    // `submit_review` (reputation zome), not "leave_feedback".
    let review_input = SubmitReviewInputMirror {
        transaction_hash: created.transaction_hash,
        listing_hash: listing_output.listing_hash,
        seller: seller.agent_pubkey.clone(),
        rating: 5,
        comment: "Excellent seller! Fast shipping and great communication.".to_string(),
    };

    let review_output: ReviewOutputMirror = buyer
        .call_zome_fn("reputation", "submit_review", review_input)
        .await;

    assert!(
        !review_output.review_hash.as_ref().is_empty(),
        "Feedback should be recorded"
    );
    assert_eq!(review_output.review.rating, 5);
}

/// Test: Get agent reputation (MATL) score.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_get_reputation_score() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 1).await;

    let agent = &agents[0];

    // A brand-new agent has no MATL score yet -- get_agent_matl_score returns
    // None until the agent's first transaction lazily initializes one (see
    // update_matl_score in zomes/reputation/coordinator/src/lib.rs). The real
    // function is "get_agent_matl_score", not "get_reputation".
    let reputation: Option<MatlScoreMirror> = agent
        .call_zome_fn(
            "reputation",
            "get_agent_matl_score",
            agent.agent_pubkey.clone(),
        )
        .await;

    assert!(
        reputation.is_none(),
        "New agent should have no MATL score yet (initialized lazily on first transaction)"
    );
}

// =============================================================================
// MATL Bridge Integration Tests
// =============================================================================

/// Test: Bridge reporting on transaction outcome.
///
/// `report_to_bridge` isn't itself a callable zome function -- Bridge
/// reporting only happens as an internal side effect of
/// `complete_transaction`, and only after finance settlement succeeds first
/// (see `settle_transaction_in_finance` in
/// zomes/transactions/coordinator/src/lib.rs). Reaching that path requires
/// installing the finance + identity clusters, which
/// `test_complete_transaction_settlement_failure_keeps_delivered_status`
/// already covers (and demonstrates settlement fails closed without them).
///
/// The one Bridge-reporting code path reachable with only marketplace
/// installed is `dispute_transaction`'s: it calls
/// `bridge::report_reputation(..)` but ignores the result
/// (`let _ = bridge::report_reputation(dispute_report);`), so a dispute must
/// succeed and transition the transaction to `Disputed` even though
/// Bridge/identity/finance aren't installed here. That's what this test
/// verifies.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_report_to_bridge() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 2).await;

    let seller = &agents[0];
    let buyer = &agents[1];

    let listing_input = serde_json::json!({
        "title": "Bridge Report Item",
        "description": "Testing Bridge integration",
        "price_cents": 5000,
        "category": "Other",
        "photos_ipfs_cids": [VALID_CID],
        "quantity_available": 1
    });

    let listing_output: ListingOutputMirror = seller
        .call_zome_fn("listings", "create_listing", listing_input)
        .await;

    wait_for_dht_sync().await;

    let created: TransactionOutputMirror = buyer
        .call_zome_fn(
            "transactions",
            "create_transaction",
            CreateTransactionInputMirror {
                seller: seller.agent_pubkey.clone(),
                listing_hash: listing_output.listing_hash,
                quantity: 1,
                total_price_cents: 5000,
            },
        )
        .await;

    wait_for_dht_sync().await;

    let disputed: TransactionOutputMirror = buyer
        .call_zome_fn(
            "transactions",
            "dispute_transaction",
            DisputeTransactionInputMirror {
                transaction_hash: created.transaction_hash,
                reason: "Item not as described".to_string(),
            },
        )
        .await;

    assert_eq!(
        disputed.transaction.status, "disputed",
        "Dispute should succeed and report to Bridge without failing, even though \
         Bridge/identity/finance aren't installed in this test"
    );
}

/// Test: Multi-agent transaction visibility via DHT.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_multi_agent_listing_visibility() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 2).await;

    let seller = &agents[0];
    let browser = &agents[1];

    // Seller creates listing
    let listing_input = serde_json::json!({
        "title": "Visibility Test Item",
        "description": "Testing DHT propagation",
        "price_cents": 1500,
        "category": "Other",
        "photos_ipfs_cids": [VALID_CID],
        "quantity_available": 1
    });

    let listing_output: ListingOutputMirror = seller
        .call_zome_fn("listings", "create_listing", listing_input)
        .await;

    wait_for_dht_sync().await;

    // Browser should see the listing
    let retrieved: Option<ListingOutputMirror> = browser
        .call_zome_fn("listings", "get_listing", listing_output.listing_hash)
        .await;

    assert!(
        retrieved.is_some(),
        "Listing should be visible to other agents via DHT"
    );
}

// =============================================================================
// Cross-Cluster Finance Settlement Tests
// =============================================================================

/// Test: `complete_transaction` invokes real finance settlement (not a no-op)
/// and correctly propagates a settlement failure by leaving the transaction
/// at `Delivered` (not `Completed`), rather than silently completing unpaid.
///
/// Deliberately installs marketplace + finance WITHOUT identity: finance's
/// `process_payment` requires the caller to meet the Participant consciousness
/// tier via a cross-cluster call to identity's `check_participant_tier`, and
/// fails closed ("Identity cluster unreachable") when identity isn't
/// reachable. A fresh test agent has no consciousness-tier history anyway
/// (combined score 0.0, below the Participant threshold of 0.3), so this
/// setup exercises exactly the failure path a brand-new, ungated caller
/// would hit in production — without needing to bootstrap a full 8D
/// sovereign-profile history just to prove the failure-handling code works.
/// See MYCELIX_REVIEW.md P1 #4 for the settlement wiring this validates.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore = "requires compiled marketplace + finance WASM and packed DNAs"]
async fn test_complete_transaction_settlement_failure_keeps_delivered_status() {
    let role_dnas: Vec<(&'static str, std::path::PathBuf)> = vec![
        ("marketplace", DnaPaths::marketplace()),
        ("finance", DnaPaths::finance()),
    ];
    if !role_dnas.iter().all(|(_, p)| p.exists()) {
        eprintln!("Skipping: marketplace and/or finance DNA not built");
        return;
    }

    let agents = setup_test_agents_from_roles(&role_dnas, 2).await;
    let seller = &agents[0];
    let buyer = &agents[1];

    // listing_hash isn't validated against a real listing by create_transaction,
    // so an arbitrary hash is fine here.
    let fake_listing_hash = ActionHash::from_raw_36(vec![9u8; 36]);

    let create_input = CreateTransactionInputMirror {
        seller: seller.agent_pubkey.clone(),
        listing_hash: fake_listing_hash,
        quantity: 1,
        total_price_cents: 5000,
    };
    let created: TransactionOutputMirror = buyer
        .call_zome_fn("transactions", "create_transaction", create_input)
        .await;

    wait_for_dht_sync().await;

    // Each mutating call creates a new action (update_entry), so the
    // response's transaction_hash must be threaded forward into the next
    // call — get_transaction(hash) fetches that EXACT action's snapshot,
    // not the latest in the update chain, so reusing a stale hash would
    // read pre-transition state.
    let confirmed: TransactionOutputMirror = seller
        .call_zome_fn(
            "transactions",
            "confirm_transaction",
            created.transaction_hash.clone(),
        )
        .await;

    wait_for_dht_sync().await;

    let mark_shipped_input = MarkShippedInputMirror {
        transaction_hash: confirmed.transaction_hash.clone(),
        tracking_info: None,
    };
    let shipped: TransactionOutputMirror = seller
        .call_zome_fn("transactions", "mark_shipped", mark_shipped_input)
        .await;

    wait_for_dht_sync().await;

    let delivered: TransactionOutputMirror = buyer
        .call_zome_fn(
            "transactions",
            "confirm_delivery",
            shipped.transaction_hash.clone(),
        )
        .await;

    wait_for_dht_sync().await;

    // complete_transaction must be called by the BUYER: finance's
    // verify_caller_is_did(&input.from_did) requires the actual calling
    // agent to equal from_did, and settle_transaction_in_finance sets
    // from_did = buyer's DID.
    let complete_result: Result<TransactionOutputMirror, _> = buyer
        .call_zome_fn_fallible(
            "transactions",
            "complete_transaction",
            delivered.transaction_hash.clone(),
        )
        .await;

    assert!(
        complete_result.is_err(),
        "complete_transaction should fail when finance settlement is rejected \
         (no identity cluster installed => consciousness gate fails closed), \
         got: {:?}",
        complete_result
    );
    let err_msg = format!("{:?}", complete_result.unwrap_err());
    assert!(
        err_msg.contains("settlement") || err_msg.contains("Delivered"),
        "Error should mention settlement failure, got: {}",
        err_msg
    );

    // The transaction must remain at Delivered (unchanged, retriable) —
    // NOT silently advance to Completed despite the failed settlement.
    // complete_transaction returned Err before calling update_entry, so no
    // new action was created — delivered.transaction_hash is still latest.
    let after: Option<TransactionOutputMirror> = buyer
        .call_zome_fn(
            "transactions",
            "get_transaction",
            delivered.transaction_hash,
        )
        .await;
    let status = after
        .expect("transaction should still exist")
        .transaction
        .status;
    assert_eq!(
        status, "delivered",
        "Transaction should remain Delivered after a failed settlement, got: {:?}",
        status
    );
}

// =============================================================================
// Mirror types for zome wire shapes
// =============================================================================
//
// Must match the real zome structs' field names/types exactly (order doesn't
// matter -- see the module doc comment above on `.with_struct_map()`).

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct EpistemicClassificationMirror {
    empirical: String,
    normative: String,
    materiality: String,
}

// ----- listings zome -----

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ListingMirror {
    title: String,
    description: String,
    price_cents: u64,
    category: String,
    photos_ipfs_cids: Vec<String>,
    quantity_available: u32,
    status: String,
    epistemic: EpistemicClassificationMirror,
    created_at: Timestamp,
    updated_at: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ListingOutputMirror {
    listing_hash: ActionHash,
    listing: ListingMirror,
    seller_agent_id: AgentPubKey,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ListingsResponseMirror {
    listings: Vec<ListingOutputMirror>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct UpdateListingInputMirror {
    listing_hash: ActionHash,
    title: Option<String>,
    description: Option<String>,
    price_cents: Option<u64>,
    category: Option<String>,
    photos_ipfs_cids: Option<Vec<String>>,
    quantity_available: Option<u32>,
    status: Option<String>,
}

// ----- transactions zome -----

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateTransactionInputMirror {
    seller: AgentPubKey,
    listing_hash: ActionHash,
    quantity: u32,
    total_price_cents: u64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct MarkShippedInputMirror {
    transaction_hash: ActionHash,
    tracking_info: Option<String>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct DisputeTransactionInputMirror {
    transaction_hash: ActionHash,
    reason: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct TransactionMirror {
    buyer: AgentPubKey,
    seller: AgentPubKey,
    listing_hash: ActionHash,
    quantity: u32,
    total_price_cents: u64,
    status: String,
    created_at: Timestamp,
    updated_at: Timestamp,
    tracking_info: Option<String>,
    epistemic: EpistemicClassificationMirror,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct TransactionOutputMirror {
    transaction_hash: ActionHash,
    transaction: TransactionMirror,
}

// ----- reputation zome -----

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SubmitReviewInputMirror {
    transaction_hash: ActionHash,
    listing_hash: ActionHash,
    seller: AgentPubKey,
    rating: u8,
    comment: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ReviewMirror {
    transaction_hash: ActionHash,
    listing_hash: ActionHash,
    rating: u8,
    comment: String,
    reviewer: AgentPubKey,
    seller: AgentPubKey,
    created_at: Timestamp,
    epistemic: EpistemicClassificationMirror,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ReviewOutputMirror {
    review_hash: ActionHash,
    review: ReviewMirror,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ProofOfGradientQualityMirror {
    quality: f64,
    consistency: f64,
    entropy: f64,
    timestamp: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ByzantineFlagsMirror {
    cartel_detected: bool,
    volatile_reputation: bool,
    gradient_poisoning: bool,
    sybil_suspected: bool,
    risk_score: f64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct MatlScoreMirror {
    agent: AgentPubKey,
    pogq: ProofOfGradientQualityMirror,
    reputation: f64,
    composite: f64,
    transaction_count: u32,
    total_value_cents: u64,
    updated_at: Timestamp,
    flags: ByzantineFlagsMirror,
}
