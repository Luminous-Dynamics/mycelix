// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Governed database epoch, recovery, and authority-delivery acknowledgement API.

use axum::{
    Extension, Json,
    extract::{Path, State},
    http::StatusCode,
};
use mycelix_desci_core::{
    SignedAuthorityDatabaseEpochCertificate, SignedAuthorityDeliveryAcknowledgement,
    SignedAuthorityRecoveryReconciliation,
};
use serde_json::json;
use std::sync::Arc;
use uuid::Uuid;

use crate::{
    error::{ApiError, Result},
    middleware::AuthenticatedActor,
    state::AppState,
};

fn postgres_store(state: &AppState) -> Result<&mycelix_desci_core::PostgresAuthorityStore> {
    state.postgres_authority_backend.as_deref().ok_or_else(|| {
        ApiError::not_implemented(
            "database epoch authority requires DESCI_SCIENTIFIC_EVENT_BACKEND=postgres",
        )
    })
}

pub async fn get_authority_database_epoch_summary(
    State(state): State<Arc<AppState>>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let governance = state.credential_governance.projection().await;
    let summary = postgres_store(&state)?
        .database_epoch_summary(&governance)
        .await?;
    Ok(Json(json!({ "summary": summary })))
}

pub async fn get_authority_database_state_commitment(
    State(state): State<Arc<AppState>>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let governance = state.credential_governance.projection().await;
    let checkpoint = governance.checkpoints.last().ok_or_else(|| {
        ApiError::invalid_request(
            "a governed transparency checkpoint must be published before database promotion",
        )
    })?;
    let checkpoint_hash = checkpoint.checkpoint_hash();
    let commitment = postgres_store(&state)?
        .authority_state_commitment(checkpoint_hash)
        .await?;
    let state_commitment_hash = commitment.commitment_hash()?;
    Ok(Json(json!({
        "checkpoint": checkpoint,
        "checkpoint_hash": checkpoint_hash,
        "state_commitment": commitment,
        "state_commitment_hash": state_commitment_hash,
        "instructions": "quiesce authority writes, obtain threshold authorization for the exact promotion intent, sign a certificate over this exact commitment, then submit it before any committed authority state changes",
    })))
}

pub async fn get_authority_database_epoch(
    State(state): State<Arc<AppState>>,
    Path(epoch_number): Path<u64>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let store = postgres_store(&state)?;
    let record = store
        .database_epoch(epoch_number)
        .await?
        .ok_or_else(|| ApiError::not_found("authority database epoch"))?;
    let reconciliations = store.recovery_reconciliations(record.epoch_hash).await?;
    Ok(Json(json!({
        "record": record,
        "reconciliations": reconciliations,
    })))
}

pub async fn record_authority_database_epoch(
    State(state): State<Arc<AppState>>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(certificate): Json<SignedAuthorityDatabaseEpochCertificate>,
) -> Result<(StatusCode, Json<serde_json::Value>)> {
    state.synchronize_authority_state().await?;
    if actor.subject() != certificate.certificate.intent.operator_actor.as_str() {
        return Err(ApiError::forbidden(
            "JWT subject must exactly match the governed database-promotion operator",
        ));
    }
    let registry = state.scientific_credentials.projection().await;
    let governance = state.credential_governance.projection().await;
    let store = postgres_store(&state)?;
    let record = store
        .record_database_epoch(certificate, &registry, &governance)
        .await?;
    let summary = store.database_epoch_summary(&governance).await?;
    Ok((
        StatusCode::CREATED,
        Json(json!({
            "record": record,
            "summary": summary,
            "publication": "queued_in_transactional_outbox",
        })),
    ))
}

pub async fn record_authority_recovery_reconciliation(
    State(state): State<Arc<AppState>>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(reconciliation): Json<SignedAuthorityRecoveryReconciliation>,
) -> Result<(StatusCode, Json<serde_json::Value>)> {
    state.synchronize_authority_state().await?;
    if actor.subject() != reconciliation.reconciliation.operator_actor.as_str() {
        return Err(ApiError::forbidden(
            "JWT subject must exactly match the signed recovery operator",
        ));
    }
    let governance = state.credential_governance.projection().await;
    let store = postgres_store(&state)?;
    let record = store.record_recovery_reconciliation(reconciliation).await?;
    let summary = store.database_epoch_summary(&governance).await?;
    Ok((
        StatusCode::CREATED,
        Json(json!({
            "record": record,
            "summary": summary,
            "publication": "queued_in_transactional_outbox",
        })),
    ))
}

pub async fn record_authority_delivery_acknowledgement(
    State(state): State<Arc<AppState>>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(acknowledgement): Json<SignedAuthorityDeliveryAcknowledgement>,
) -> Result<(StatusCode, Json<serde_json::Value>)> {
    state.synchronize_authority_state().await?;
    if actor.subject() != acknowledgement.acknowledgement.actor.as_str() {
        return Err(ApiError::forbidden(
            "JWT subject must exactly match the signed delivery-acknowledgement witness",
        ));
    }
    let governance = state.credential_governance.projection().await;
    let store = postgres_store(&state)?;
    let record = store
        .record_delivery_acknowledgement(acknowledgement, &governance)
        .await?;
    Ok((StatusCode::CREATED, Json(json!({ "record": record }))))
}

pub async fn get_authority_delivery_acknowledgements(
    State(state): State<Arc<AppState>>,
    Path(delivery_id): Path<Uuid>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let governance = state.credential_governance.projection().await;
    let records = postgres_store(&state)?
        .delivery_acknowledgements(delivery_id)
        .await?;
    let rendered = records
        .into_iter()
        .map(|record| {
            let verification = record
                .acknowledgement
                .verify_against_governance(&governance);
            json!({
                "currently_valid": verification.is_ok(),
                "verification_error": verification.err().map(|error| error.to_string()),
                "record": record,
            })
        })
        .collect::<Vec<_>>();
    Ok(Json(json!({
        "delivery_id": delivery_id,
        "acknowledgements": rendered,
    })))
}
