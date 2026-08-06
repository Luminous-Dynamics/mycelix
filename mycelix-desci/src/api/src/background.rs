// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Durable background publication workers.

use crate::state::AppState;
use std::sync::Arc;
use std::time::Duration;
use tracing::{error, info, warn};
use uuid::Uuid;

/// Keep the archival worker disabled until a real archive adapter is configured.
pub fn start_quantum_anchor(_state: Arc<AppState>) {
    warn!(
        "Quantum Anchor worker is disabled: no authenticated proof source or Arweave receipt adapter is configured"
    );
}

/// Publish PostgreSQL authority-outbox records to an operator-controlled HTTP
/// endpoint. The database lease makes multiple API replicas safe: only one
/// worker owns a row at a time, failed deliveries become retryable, and success
/// is acknowledged only after a 2xx response.
pub fn start_authority_outbox_publisher(state: Arc<AppState>) {
    let Some(store) = state.postgres_authority_backend.clone() else {
        return;
    };
    let endpoint = match std::env::var("DESCI_AUTHORITY_OUTBOX_WEBHOOK_URL") {
        Ok(value) if !value.trim().is_empty() => value,
        _ => {
            warn!(
                "PostgreSQL authority outbox is durable but no DESCI_AUTHORITY_OUTBOX_WEBHOOK_URL is configured; records will remain pending"
            );
            return;
        }
    };
    let endpoint_url = match reqwest::Url::parse(&endpoint) {
        Ok(url) => url,
        Err(error) => {
            error!(%error, "invalid DESCI_AUTHORITY_OUTBOX_WEBHOOK_URL; publisher disabled");
            return;
        }
    };
    let allow_insecure_http = std::env::var("DESCI_AUTHORITY_OUTBOX_ALLOW_INSECURE_HTTP")
        .is_ok_and(|value| matches!(value.to_ascii_lowercase().as_str(), "1" | "true" | "yes"));
    if endpoint_url.host_str().is_none()
        || endpoint_url.username() != ""
        || endpoint_url.password().is_some()
        || endpoint_url.fragment().is_some()
        || endpoint_url.query().is_some()
        || (endpoint_url.scheme() != "https"
            && !(allow_insecure_http && endpoint_url.scheme() == "http"))
    {
        error!(
            "authority outbox endpoint must be an absolute HTTPS URL without credentials, query parameters, or fragments; publisher disabled"
        );
        return;
    }
    let worker_id = std::env::var("DESCI_AUTHORITY_OUTBOX_WORKER_ID")
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| format!("mycelix-api-{}", Uuid::new_v4()));
    let batch_size = parse_bounded_env("DESCI_AUTHORITY_OUTBOX_BATCH_SIZE", 50, 1, 1_000);
    let lease_seconds = parse_bounded_env("DESCI_AUTHORITY_OUTBOX_LEASE_SECONDS", 60, 1, 3_600);
    let poll_millis = parse_bounded_env("DESCI_AUTHORITY_OUTBOX_POLL_MILLIS", 1_000, 100, 60_000);
    let client = match reqwest::Client::builder()
        .redirect(reqwest::redirect::Policy::none())
        .timeout(Duration::from_secs(30))
        .build()
    {
        Ok(client) => client,
        Err(error) => {
            error!(%error, "failed to construct authority outbox HTTP client; publisher disabled");
            return;
        }
    };

    tokio::spawn(async move {
        info!(%worker_id, endpoint = %endpoint_url, "starting authority outbox publisher");
        loop {
            match store
                .claim_outbox_batch(&worker_id, batch_size as usize, lease_seconds)
                .await
            {
                Ok(messages) if messages.is_empty() => {}
                Ok(messages) => {
                    for message in messages {
                        let response = client
                            .post(endpoint_url.clone())
                            .header("content-type", "application/json")
                            .header("x-mycelix-outbox-id", message.id.to_string())
                            .header("x-mycelix-delivery-hash", message.delivery_hash.to_string())
                            .header("idempotency-key", message.id.to_string())
                            .json(&message.delivery)
                            .send()
                            .await;
                        match response {
                            Ok(response) if response.status().is_success() => {
                                if let Err(error) = store
                                    .mark_outbox_published(
                                        message.id,
                                        &worker_id,
                                        chrono::Utc::now(),
                                    )
                                    .await
                                {
                                    error!(outbox_id = %message.id, %error, "failed to acknowledge published outbox row");
                                }
                            }
                            Ok(response) => {
                                let detail =
                                    format!("webhook returned HTTP {}", response.status().as_u16());
                                if let Err(error) = store
                                    .mark_outbox_failed(message.id, &worker_id, &detail)
                                    .await
                                {
                                    error!(outbox_id = %message.id, %error, "failed to release rejected outbox row");
                                }
                            }
                            Err(delivery_error) => {
                                if let Err(error) = store
                                    .mark_outbox_failed(
                                        message.id,
                                        &worker_id,
                                        &delivery_error.to_string(),
                                    )
                                    .await
                                {
                                    error!(outbox_id = %message.id, %error, "failed to release undelivered outbox row");
                                }
                            }
                        }
                    }
                }
                Err(error) => {
                    error!(%error, "authority outbox poll failed");
                }
            }
            tokio::time::sleep(Duration::from_millis(poll_millis)).await;
        }
    });
}

fn parse_bounded_env(name: &str, default: u64, min: u64, max: u64) -> u64 {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| (*value >= min) && (*value <= max))
        .unwrap_or(default)
}
