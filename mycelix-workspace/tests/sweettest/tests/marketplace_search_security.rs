// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Marketplace search + security zome sweettest integration tests.
//!
//! Regression coverage for the EntryHash/ActionHash link-target bug fixed in
//! cbd1d4ab9a (search/coordinator's create_search_index, security/
//! coordinator's log_security_event) -- the same class of bug found and
//! fixed in the listings zome (c24979bba1). Both fixes were previously
//! unverified behaviorally since neither zome had any sweettest coverage;
//! this file closes that gap.
//!
//! Prerequisites:
//!   cd mycelix-marketplace/backend && cargo build --release --target wasm32-unknown-unknown
//!   hc dna pack .
//!
//! Run: cargo test --release -p mycelix-sweettest --test marketplace_search_security -- --ignored
//!
//! See marketplace_workflow.rs's module doc comment for why HoloHash-bearing
//! zome structs need typed "Mirror" structs rather than serde_json::Value.

mod harness;

use harness::*;
use holochain::prelude::*;
use serial_test::serial;
use std::collections::HashMap;

// =============================================================================
// Search zome tests
// =============================================================================

/// Test: creating a search index makes the entity discoverable via `search`.
///
/// Exercises the `AllIndices` link, which `search()` reads via
/// `get_all_indices_links()` -> `link.target.into_action_hash()`. Before the
/// fix, create_search_index's links all targeted an EntryHash, so this link
/// was silently invisible and `search` always returned zero results
/// regardless of how long you waited for DHT sync.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_create_and_search_index() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 1).await;
    let seller = &agents[0];

    // A fake listing action hash to index against -- create_search_index
    // doesn't validate that entity_hash actually resolves to a real entry.
    let fake_listing_hash = fake_action_hash();

    let create_input = CreateIndexInputMirror {
        entity_type: "Listing".to_string(),
        entity_hash: fake_listing_hash.clone(),
        title: "Handwoven Ceramic Vase".to_string(),
        description: "A unique handwoven ceramic vase, glazed in blue.".to_string(),
        tags: vec!["ceramic".to_string(), "handmade".to_string()],
        category: "Home & Garden".to_string(),
        creator_matl_score: 0.9,
        price_cents: Some(4500),
        status: "active".to_string(),
    };

    let _index_hash: ActionHash = seller
        .call_zome_fn("search", "create_search_index", create_input)
        .await;

    let mut response = SearchResponseMirror {
        results: vec![],
        total_count: 0,
        query_time_ms: 0,
    };
    for _ in 0..15 {
        let query = SearchQueryMirror {
            query: "ceramic vase".to_string(),
            entity_types: None,
            limit: 10,
        };
        response = seller.call_zome_fn("search", "search", query).await;
        if response.total_count > 0 {
            break;
        }
        wait_for_dht_sync().await;
    }

    assert_eq!(
        response.total_count, 1,
        "search should find the indexed listing via the AllIndices link"
    );
    assert_eq!(response.results[0].entity_hash, fake_listing_hash);
    assert_eq!(response.results[0].title, "Handwoven Ceramic Vase");
}

/// Test: `get_entity_index` retrieves the index for a specific entity.
///
/// Exercises the `EntityToIndex` link, keyed by entity_hash directly (not a
/// Path anchor) -- was equally broken before the fix (EntryHash-targeted),
/// since `get_entity_index` also calls `link.target.into_action_hash()`.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_get_entity_index() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 1).await;
    let seller = &agents[0];

    let fake_transaction_hash = fake_action_hash();

    let create_input = CreateIndexInputMirror {
        entity_type: "Transaction".to_string(),
        entity_hash: fake_transaction_hash.clone(),
        title: "Transaction Record".to_string(),
        description: "A completed transaction.".to_string(),
        tags: vec![],
        category: "general".to_string(),
        creator_matl_score: 0.8,
        price_cents: None,
        status: "completed".to_string(),
    };

    let _index_hash: ActionHash = seller
        .call_zome_fn("search", "create_search_index", create_input)
        .await;

    let mut found: Option<SearchIndexEntryMirror> = None;
    for _ in 0..15 {
        found = seller
            .call_zome_fn("search", "get_entity_index", fake_transaction_hash.clone())
            .await;
        if found.is_some() {
            break;
        }
        wait_for_dht_sync().await;
    }

    let entry = found.expect("get_entity_index should find the index via the EntityToIndex link");
    assert_eq!(entry.entity_hash, fake_transaction_hash);
    assert_eq!(entry.title, "Transaction Record");
}

// =============================================================================
// Security zome tests
// =============================================================================

/// Test: logging a security event makes it visible via all three discovery
/// paths (agent, all-logs anchor, event-type anchor).
///
/// Exercises AgentToSecurityLogs, AllSecurityLogs, and EventTypeToLogs --
/// all three were EntryHash-targeted before the fix, so
/// get_my_security_logs/get_all_security_logs/get_logs_by_event_type all
/// returned zero results regardless of DHT sync wait time.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_log_and_retrieve_security_event() {
    let agents = setup_test_agents(&DnaPaths::marketplace(), "mycelix-marketplace", 1).await;
    let agent = &agents[0];

    let log_input = CreateSecurityLogInputMirror {
        event_type: "rate_limit_exceeded".to_string(),
        severity: "High".to_string(),
        zome_name: "listings".to_string(),
        function_name: "create_listing".to_string(),
        description: "Agent exceeded listing creation rate limit".to_string(),
        trigger_input: Some("burst of 50 listings in 10s".to_string()),
        metadata: "{}".to_string(),
    };

    let _log_hash: ActionHash = agent
        .call_zome_fn("security", "log_security_event", log_input)
        .await;

    // 1. AgentToSecurityLogs
    let mut my_logs = SecurityLogsResponseMirror {
        logs: vec![],
        total_count: 0,
    };
    for _ in 0..15 {
        my_logs = agent
            .call_zome_fn("security", "get_my_security_logs", ())
            .await;
        if my_logs.total_count > 0 {
            break;
        }
        wait_for_dht_sync().await;
    }
    assert_eq!(
        my_logs.total_count, 1,
        "get_my_security_logs should find the log via AgentToSecurityLogs"
    );

    // 2. AllSecurityLogs
    let all_logs: SecurityLogsResponseMirror = agent
        .call_zome_fn("security", "get_all_security_logs", ())
        .await;
    assert_eq!(
        all_logs.total_count, 1,
        "get_all_security_logs should find the log via AllSecurityLogs"
    );

    // 3. EventTypeToLogs
    let by_type: SecurityLogsResponseMirror = agent
        .call_zome_fn(
            "security",
            "get_logs_by_event_type",
            "rate_limit_exceeded".to_string(),
        )
        .await;
    assert_eq!(
        by_type.total_count, 1,
        "get_logs_by_event_type should find the log via EventTypeToLogs"
    );
    assert_eq!(
        by_type.logs[0].log.description,
        "Agent exceeded listing creation rate limit"
    );
}

/// A deterministic, valid-shape ActionHash for tests that only need a stable
/// identifier to index against, not a real DHT entry.
fn fake_action_hash() -> ActionHash {
    ActionHash::from_raw_36(vec![0xAB; 36])
}

// =============================================================================
// Mirror types for zome wire shapes
// =============================================================================
//
// Must match the real zome structs' field names/types exactly (order doesn't
// matter -- see marketplace_workflow.rs's module doc comment on
// `.with_struct_map()`). Enum fields are mirrored as String, matching each
// enum's serde rename_all convention (EntityType/ListingCategory-style
// enums: PascalCase; SecurityEventType: snake_case; SecuritySeverity: no
// rename, plain Rust variant names).

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateIndexInputMirror {
    entity_type: String,
    entity_hash: ActionHash,
    title: String,
    description: String,
    tags: Vec<String>,
    category: String,
    creator_matl_score: f64,
    price_cents: Option<u64>,
    status: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SearchQueryMirror {
    query: String,
    entity_types: Option<Vec<String>>,
    limit: u32,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SearchResultMirror {
    entity_hash: ActionHash,
    entity_type: String,
    title: String,
    description_snippet: String,
    score: f64,
    creator: AgentPubKey,
    created_at: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SearchResponseMirror {
    results: Vec<SearchResultMirror>,
    total_count: u32,
    query_time_ms: u64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SearchIndexEntryMirror {
    index_id: String,
    entity_type: String,
    entity_hash: ActionHash,
    title: String,
    description: String,
    tags: Vec<String>,
    category: String,
    creator: AgentPubKey,
    creator_matl_score: f64,
    created_at: Timestamp,
    updated_at: Timestamp,
    price_cents: Option<u64>,
    status: String,
    processed_terms: Vec<String>,
    term_frequencies: HashMap<String, u32>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateSecurityLogInputMirror {
    event_type: String,
    severity: String,
    zome_name: String,
    function_name: String,
    description: String,
    trigger_input: Option<String>,
    metadata: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SecurityLogMirror {
    event_type: String,
    severity: String,
    zome_name: String,
    function_name: String,
    agent: AgentPubKey,
    description: String,
    trigger_input: Option<String>,
    timestamp: Timestamp,
    metadata: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SecurityLogOutputMirror {
    log_hash: ActionHash,
    log: SecurityLogMirror,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SecurityLogsResponseMirror {
    logs: Vec<SecurityLogOutputMirror>,
    total_count: u32,
}
