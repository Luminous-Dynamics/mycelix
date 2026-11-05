// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Integration tests for transaction-time canonical claim projections.

use super::helpers::TestServer;
use uuid::Uuid;

#[tokio::test]
async fn as_known_at_requires_an_explicit_timestamp() {
    let server = TestServer::start()
        .await
        .expect("test server should start");
    let claim_id = Uuid::new_v4();

    let response = server
        .get(&format!(
            "/scientific/claims/{claim_id}/as-known-at"
        ))
        .await;

    assert_eq!(
        response.status(),
        reqwest::StatusCode::BAD_REQUEST,
        "the API must not silently substitute occurred_at or current time"
    );
}

#[tokio::test]
async fn as_known_at_returns_not_found_for_an_unknown_claim() {
    let server = TestServer::start()
        .await
        .expect("test server should start");
    let claim_id = Uuid::new_v4();

    let response = server
        .get(&format!(
            "/scientific/claims/{claim_id}/as-known-at?known_at=2026-11-05T17%3A00%3A00Z"
        ))
        .await;

    assert_eq!(response.status(), reqwest::StatusCode::NOT_FOUND);
}
