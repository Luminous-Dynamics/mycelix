// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! System API handlers

use axum::{Json, extract::State, http::StatusCode};
use std::{collections::BTreeSet, sync::Arc};
use tracing::{info, instrument};

use crate::{error::Result, models::*, state::AppState};

/// Health check endpoint
#[utoipa::path(
    get,
    path = "/api/v1/system/health",
    responses(
        (status = 200, description = "Service is healthy", body = HealthResponse),
        (status = 503, description = "Service is unhealthy", body = HealthResponse),
    ),
    tag = "system"
)]
#[instrument(skip(state))]
pub async fn health_check(
    State(state): State<Arc<AppState>>,
) -> Result<(StatusCode, Json<HealthResponse>)> {
    info!("Health check");
    state.synchronize_authority_state().await?;

    // Canonical writes are not ready until identity authority, durable event
    // storage, and receipt signing are configured. Pre-v0.5 canonical history
    // remains explicitly unattested unless an operator acknowledges the cutover.
    let audit = state.scientific_authority_audit.summary().await?;
    let postgres_authority_healthy = match &state.postgres_authority_backend {
        Some(store) => store.health_check().await.is_ok(),
        None => true,
    };
    let authority_write_fencing = match &state.postgres_authority_backend {
        Some(store) => Some(store.write_fencing_status().await),
        None => None,
    };
    let authority_write_fencing_acceptable = if !state.require_authority_write_fencing {
        true
    } else {
        authority_write_fencing
            .as_ref()
            .is_some_and(|status| status.configured && status.active)
    };
    let postgres_outbox = match &state.postgres_authority_backend {
        Some(store) => Some(store.outbox_summary().await?),
        None => None,
    };
    let oldest_outbox_age_seconds = postgres_outbox
        .as_ref()
        .and_then(|summary| summary.oldest_pending_at.as_ref())
        .map(|oldest| (chrono::Utc::now() - oldest.clone()).num_seconds().max(0));
    let outbox_policy_acceptable = postgres_outbox.as_ref().is_none_or(|summary| {
        (state.maximum_pending_authority_outbox == 0
            || summary.pending <= state.maximum_pending_authority_outbox)
            && (state.maximum_authority_outbox_age_seconds == 0
                || oldest_outbox_age_seconds
                    .is_none_or(|age| age <= state.maximum_authority_outbox_age_seconds))
    });
    let credentials = state.scientific_credentials.summary().await;
    let credential_governance = state.credential_governance.summary().await;
    let governance_projection = state.credential_governance.projection().await;
    let database_epoch_summary = match &state.postgres_authority_backend {
        Some(store) => Some(store.database_epoch_summary(&governance_projection).await?),
        None => None,
    };
    let witness_policy_acceptable = state.minimum_checkpoint_witness_organizations == 0
        || (credential_governance.latest_checkpoint_hash.is_some()
            && credential_governance.latest_checkpoint_witness_organizations
                >= state.minimum_checkpoint_witness_organizations);
    let latest_checkpoint_mirror_organizations = match (
        credential_governance.latest_checkpoint_hash,
        state.postgres_authority_backend.as_ref(),
    ) {
        (Some(checkpoint_hash), Some(store)) => {
            let observations = store
                .checkpoint_mirror_observations(checkpoint_hash)
                .await?;
            observations
                .into_iter()
                .filter(|record| {
                    record
                        .observation
                        .verify_against_governance(&governance_projection)
                        .is_ok()
                })
                .map(|record| record.observation.organization)
                .collect::<BTreeSet<_>>()
                .len()
        }
        _ => 0,
    };
    let mirror_policy_acceptable = state.minimum_checkpoint_mirror_organizations == 0
        || latest_checkpoint_mirror_organizations >= state.minimum_checkpoint_mirror_organizations;
    let database_epoch_policy_acceptable = if !state.require_authority_database_epoch {
        true
    } else {
        database_epoch_summary.as_ref().is_some_and(|summary| {
            summary.epoch_count > 0
                && summary.latest_epoch_hash.is_some()
                && summary.latest_published_delivery_id.is_some()
                && summary.latest_promotion_mode.is_none_or(|mode| {
                    mode != mycelix_desci_core::DatabasePromotionMode::DisasterRecovery
                        || summary.latest_epoch_reconciled
                })
                && summary.latest_delivery_acknowledgement_organizations
                    >= state.minimum_database_epoch_acknowledgement_organizations
        })
    };
    let governance_acceptable = credential_governance.initialized
        && credential_governance.risk_policy_configured
        && credential_governance.durable
        && credential_governance.acceptance_signing_configured
        && credential_governance.current_acceptance_signing_key_active
        && credential_governance.active_acceptance_service_keys > 0
        && credential_governance.continuing_acceptance_service_keys > 0
        && !credential_governance.pending_execution_recovery
        && credential_governance
            .routine_approval_threshold
            .is_some_and(|threshold| {
                usize::from(threshold) >= state.minimum_credential_registry_admins
            })
        && witness_policy_acceptable
        && mirror_policy_acceptable;
    let credential_registry_acceptable = credentials.initialized
        && credentials.durable
        && credentials.acceptance_signing_configured
        && credentials.active_registry_administrators >= state.minimum_credential_registry_admins
        && credentials.continuing_registry_administrators >= 1;
    let legacy_history_acceptable =
        audit.legacy_unattested_events == 0 || state.allow_unattested_authority_history;
    let authority_history_acceptable =
        legacy_history_acceptable && audit.unsafe_unattested_events == 0;
    let overall_healthy = credential_registry_acceptable
        && governance_acceptable
        && state.authority_receipt_signing_configured
        && (state.postgres_authority_backend.is_none()
            || state.authority_delivery_signing_configured)
        && state.scientific_events_durable
        && postgres_authority_healthy
        && authority_write_fencing_acceptable
        && database_epoch_policy_acceptable
        && outbox_policy_acceptable
        && audit.pending_receipts == 0
        && authority_history_acceptable;

    let response = HealthResponse {
        status: if overall_healthy {
            "healthy"
        } else {
            "degraded"
        }
        .to_string(),
        version: env!("CARGO_PKG_VERSION").to_string(),
        uptime_seconds: state.uptime().as_secs(),
        checks: vec![
            HealthCheck {
                name: "storage".to_string(),
                status: "passing".to_string(),
            },
            HealthCheck {
                name: "query_engine".to_string(),
                status: "passing".to_string(),
            },
            HealthCheck {
                name: "postgres_authority_transaction".to_string(),
                status: if state.postgres_authority_backend.is_none() {
                    "not_selected"
                } else if postgres_authority_healthy {
                    "passing"
                } else {
                    "failing"
                }
                .to_string(),
            },
            HealthCheck {
                name: "authority_write_fencing".to_string(),
                status: if state.postgres_authority_backend.is_none() {
                    "not_selected".to_string()
                } else if !state.require_authority_write_fencing {
                    "not_required".to_string()
                } else {
                    match &authority_write_fencing {
                        Some(status) if status.active => format!(
                            "passing:generation:{}_primary:{}_timeline:{}_expires:{}",
                            status.generation.unwrap_or_default(),
                            status.primary_id.as_deref().unwrap_or("unknown"),
                            status.postgres_timeline.unwrap_or_default(),
                            status
                                .expires_at
                                .map(|value| value.to_rfc3339())
                                .unwrap_or_else(|| "unknown".to_string())
                        ),
                        Some(status) => format!(
                            "blocked:{}",
                            status.error.as_deref().unwrap_or("inactive_write_lease")
                        ),
                        None => "blocked:not_configured".to_string(),
                    }
                },
            },
            HealthCheck {
                name: "authority_database_epoch".to_string(),
                status: if !state.require_authority_database_epoch {
                    "not_required".to_string()
                } else {
                    match &database_epoch_summary {
                        None => "blocked:postgres_not_selected".to_string(),
                        Some(summary) if summary.epoch_count == 0 => {
                            "blocked:no_governed_database_epoch".to_string()
                        }
                        Some(summary) if summary.latest_published_delivery_id.is_none() => {
                            "blocked:epoch_publication_pending".to_string()
                        }
                        Some(summary)
                            if summary.latest_promotion_mode
                                == Some(
                                    mycelix_desci_core::DatabasePromotionMode::DisasterRecovery,
                                )
                                && !summary.latest_epoch_reconciled =>
                        {
                            "blocked:disaster_recovery_not_reconciled".to_string()
                        }
                        Some(summary)
                            if summary.latest_delivery_acknowledgement_organizations
                                < state.minimum_database_epoch_acknowledgement_organizations =>
                        {
                            format!(
                                "blocked:epoch_delivery_ack_organizations:{}/{}",
                                summary.latest_delivery_acknowledgement_organizations,
                                state.minimum_database_epoch_acknowledgement_organizations
                            )
                        }
                        Some(summary) => format!(
                            "passing:epoch:{}_primary:{}_ack_orgs:{}",
                            summary.latest_epoch_number.unwrap_or_default(),
                            summary.latest_primary_id.as_deref().unwrap_or("unknown"),
                            summary.latest_delivery_acknowledgement_organizations
                        ),
                    }
                },
            },
            HealthCheck {
                name: "authority_delivery_signing".to_string(),
                status: if state.postgres_authority_backend.is_none() {
                    "not_selected".to_string()
                } else if state.authority_delivery_signing_configured {
                    format!(
                        "passing:{}",
                        state
                            .authority_delivery_public_key
                            .as_deref()
                            .unwrap_or("unknown")
                    )
                } else {
                    "failing:not_configured".to_string()
                },
            },
            HealthCheck {
                name: "authority_publication_outbox".to_string(),
                status: match &postgres_outbox {
                    Some(summary) if summary.pending == 0 => "passing:empty".to_string(),
                    Some(summary) if outbox_policy_acceptable => format!(
                        "passing:pending:{}_leased:{}_attempts:{}_max_attempts:{}_oldest_age_seconds:{}",
                        summary.pending,
                        summary.leased,
                        summary.delivery_attempts,
                        summary.maximum_attempts,
                        oldest_outbox_age_seconds.unwrap_or_default()
                    ),
                    Some(summary) => format!(
                        "blocked:pending:{}_limit:{}_oldest_age_seconds:{}_age_limit:{}_attempts:{}_max_attempts:{}",
                        summary.pending,
                        state.maximum_pending_authority_outbox,
                        oldest_outbox_age_seconds.unwrap_or_default(),
                        state.maximum_authority_outbox_age_seconds,
                        summary.delivery_attempts,
                        summary.maximum_attempts
                    ),
                    None => "not_selected".to_string(),
                },
            },
            HealthCheck {
                name: "checkpoint_mirror_policy".to_string(),
                status: if state.minimum_checkpoint_mirror_organizations == 0 {
                    "not_required".to_string()
                } else if mirror_policy_acceptable {
                    format!(
                        "passing:{}/{}",
                        latest_checkpoint_mirror_organizations,
                        state.minimum_checkpoint_mirror_organizations
                    )
                } else {
                    format!(
                        "insufficient_mirror_organizations:{}/{}",
                        latest_checkpoint_mirror_organizations,
                        state.minimum_checkpoint_mirror_organizations
                    )
                },
            },
            HealthCheck {
                name: "scientific_event_log".to_string(),
                status: if state.scientific_events_durable {
                    "durable"
                } else {
                    "ephemeral_simulation"
                }
                .to_string(),
            },
            HealthCheck {
                name: "scientific_credential_registry".to_string(),
                status: if !credentials.initialized {
                    "uninitialized".to_string()
                } else if !credentials.durable {
                    "ephemeral_simulation".to_string()
                } else if !credentials.acceptance_signing_configured {
                    "acceptance_signing_not_configured".to_string()
                } else if credentials.continuing_registry_administrators == 0 {
                    "no_continuing_admin_recovery_key".to_string()
                } else if credentials.active_registry_administrators
                    < state.minimum_credential_registry_admins
                {
                    format!(
                        "insufficient_admins:{}/{}",
                        credentials.active_registry_administrators,
                        state.minimum_credential_registry_admins
                    )
                } else {
                    format!(
                        "passing:{}_actors:{}_events:{}_continuing_admins",
                        credentials.actor_count,
                        credentials.event_count,
                        credentials.continuing_registry_administrators
                    )
                },
            },
            HealthCheck {
                name: "credential_threshold_governance".to_string(),
                status: if !credential_governance.initialized {
                    "uninitialized".to_string()
                } else if !credential_governance.risk_policy_configured {
                    "risk_policy_not_configured".to_string()
                } else if !credential_governance.durable {
                    "ephemeral_simulation".to_string()
                } else if !credential_governance.acceptance_signing_configured {
                    "acceptance_signing_not_configured".to_string()
                } else if !credential_governance.current_acceptance_signing_key_active {
                    "configured_acceptance_signing_key_is_not_active".to_string()
                } else if credential_governance.active_acceptance_service_keys == 0 {
                    "no_active_acceptance_service_key".to_string()
                } else if credential_governance.continuing_acceptance_service_keys == 0 {
                    "no_continuing_acceptance_recovery_key".to_string()
                } else if credential_governance.pending_execution_recovery {
                    "pending_execution_recovery".to_string()
                } else if !witness_policy_acceptable {
                    format!(
                        "insufficient_checkpoint_witness_organizations:{}/{}",
                        credential_governance.latest_checkpoint_witness_organizations,
                        state.minimum_checkpoint_witness_organizations
                    )
                } else if credential_governance
                    .routine_approval_threshold
                    .is_some_and(|threshold| {
                        usize::from(threshold) < state.minimum_credential_registry_admins
                    })
                {
                    format!(
                        "threshold_below_minimum:{}/{}",
                        credential_governance
                            .routine_approval_threshold
                            .unwrap_or_default(),
                        state.minimum_credential_registry_admins
                    )
                } else {
                    format!(
                        "passing:{}_events:{}_pending:{}_routine_threshold:{}_witness_orgs",
                        credential_governance.event_count,
                        credential_governance.pending_proposals,
                        credential_governance
                            .routine_approval_threshold
                            .unwrap_or_default(),
                        credential_governance.latest_checkpoint_witness_organizations
                    )
                },
            },
            HealthCheck {
                name: "authority_receipt_signing".to_string(),
                status: if state.authority_receipt_signing_configured {
                    "passing"
                } else {
                    "not_configured"
                }
                .to_string(),
            },
            HealthCheck {
                name: "authority_receipt_journal".to_string(),
                status: if audit.pending_receipts > 0 {
                    "pending_reconciliation"
                } else {
                    "passing"
                }
                .to_string(),
            },
            HealthCheck {
                name: "authority_history".to_string(),
                status: if audit.unsafe_unattested_events > 0 {
                    format!(
                        "blocked:{}_unsafe_receipt_gaps",
                        audit.unsafe_unattested_events
                    )
                } else if audit.legacy_unattested_events == 0 {
                    "fully_attested".to_string()
                } else if state.allow_unattested_authority_history {
                    format!(
                        "cutover_acknowledged:{}_legacy_unattested",
                        audit.legacy_unattested_events
                    )
                } else {
                    format!(
                        "blocked:{}_legacy_unattested",
                        audit.legacy_unattested_events
                    )
                },
            },
            HealthCheck {
                name: "legacy_mutations".to_string(),
                status: if state.legacy_mutations_enabled {
                    "enabled_noncanonical"
                } else {
                    "disabled"
                }
                .to_string(),
            },
        ],
    };

    let status_code = if overall_healthy {
        StatusCode::OK
    } else {
        StatusCode::SERVICE_UNAVAILABLE
    };

    Ok((status_code, Json(response)))
}

/// Get system metrics
#[utoipa::path(
    get,
    path = "/api/v1/system/metrics",
    responses(
        (status = 200, description = "Metrics retrieved", body = MetricsResponse),
    ),
    tag = "system"
)]
#[instrument(skip(state))]
pub async fn get_metrics(State(state): State<Arc<AppState>>) -> Result<Json<MetricsResponse>> {
    info!("Retrieving metrics");

    // TODO: Implement proper metrics aggregation
    let response = MetricsResponse {
        uptime_seconds: state.uptime().as_secs(),
        total_claims: 0,
        total_participants: 0,
        queries_executed: state
            .metrics
            .queries_executed
            .load(std::sync::atomic::Ordering::Relaxed),
        claims_created: state
            .metrics
            .claims_created
            .load(std::sync::atomic::Ordering::Relaxed),
        verifications_added: state
            .metrics
            .verifications_added
            .load(std::sync::atomic::Ordering::Relaxed),
        average_response_time_ms: state.metrics.average_response_time_ms(),
    };

    Ok(Json(response))
}

/// Get version information
#[utoipa::path(
    get,
    path = "/api/v1/system/version",
    responses(
        (status = 200, description = "Version information", body = VersionResponse),
    ),
    tag = "system"
)]
#[instrument]
pub async fn get_version() -> Result<Json<VersionResponse>> {
    info!("Retrieving version");

    let response = VersionResponse {
        version: env!("CARGO_PKG_VERSION").to_string(),
        build_date: option_env!("BUILD_DATE").unwrap_or("unknown").to_string(),
        git_commit: option_env!("GIT_COMMIT").unwrap_or("unknown").to_string(),
        rust_version: option_env!("RUSTC_VERSION")
            .unwrap_or("unknown")
            .to_string(),
    };

    Ok(Json(response))
}
