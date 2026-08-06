// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Canonical append-only scientific event API.
//!
//! These endpoints are the authoritative write surface. Clients construct and
//! sign canonical events locally; the server checks JWT/event actor equality,
//! durable actor-key binding, scientific roles, stream concurrency, and replay
//! invariants before committing an event.

use axum::{
    Extension, Json,
    extract::{Path, Query, State},
    http::StatusCode,
};
use mycelix_desci_core::{ClaimId, ClaimProjection, ScientificEventId, ScientificEventPayload};
use serde_json::json;
use std::sync::Arc;
use uuid::Uuid;

use crate::{
    error::{ApiError, Result},
    middleware::AuthenticatedActor,
    models::{AppendScientificEventRequest, ScientificEventPageQuery},
    state::AppState,
};

/// Commit a client-signed canonical event.
pub async fn append_scientific_event(
    State(state): State<Arc<AppState>>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(request): Json<AppendScientificEventRequest>,
) -> Result<(StatusCode, Json<serde_json::Value>)> {
    if actor.subject() != request.event.envelope.actor.as_str() {
        return Err(ApiError::forbidden(
            "JWT subject must exactly match the signed scientific event actor",
        ));
    }
    if matches!(
        &request.event.envelope.payload,
        ScientificEventPayload::LegacyClaimImported { .. }
    ) {
        return Err(ApiError::forbidden(
            "legacy imports are offline migration operations and cannot be submitted through the public event API",
        ));
    }

    let claim_id = request.event.envelope.stream_id;
    let receipt = state
        .scientific_events
        .append(request.expected_sequence, request.event)
        .await?;
    let authority_receipt = state
        .scientific_authority_audit
        .receipt(receipt.event_id)
        .await?
        .ok_or_else(|| {
            ApiError::internal_error(
                "scientific event committed without a finalized authority receipt",
            )
        })?;
    let stream = state.scientific_events.stream(claim_id).await?;
    let projection = ClaimProjection::rebuild(&stream)?;

    Ok((
        StatusCode::CREATED,
        Json(json!({
            "receipt": receipt,
            "authority_receipt": authority_receipt,
            "projection": projection,
        })),
    ))
}

/// Return the deterministic projection for one canonical claim stream.
pub async fn get_scientific_claim(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<serde_json::Value>> {
    let claim_id = ClaimId(id);
    let stream = state.scientific_events.stream(claim_id).await?;
    if stream.is_empty() {
        return Err(ApiError::not_found("canonical scientific claim"));
    }
    let projection = ClaimProjection::rebuild(&stream)?;
    let assessment = projection.assessment();
    Ok(Json(json!({
        "projection": projection,
        "assessment": assessment,
    })))
}

/// Page through the signed source events for one canonical claim stream.
pub async fn get_scientific_claim_events(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
    Query(query): Query<ScientificEventPageQuery>,
) -> Result<Json<serde_json::Value>> {
    let claim_id = ClaimId(id);
    if state.scientific_events.head(claim_id).await?.is_none() {
        return Err(ApiError::not_found("canonical scientific claim"));
    }
    let from_sequence = query.from_sequence.unwrap_or(0);
    let limit = query.limit.unwrap_or(100).clamp(1, 500);
    let page = state
        .scientific_events
        .read(claim_id, from_sequence, limit)
        .await?;
    Ok(Json(json!({
        "stream_id": claim_id,
        "from_sequence": from_sequence,
        "limit": limit,
        "page": page,
    })))
}

/// Return the ordered receipt-time authority chain for one claim stream.
pub async fn get_scientific_claim_authority_receipts(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<serde_json::Value>> {
    let claim_id = ClaimId(id);
    let stream = state.scientific_events.stream(claim_id).await?;
    if stream.is_empty() {
        return Err(ApiError::not_found("canonical scientific claim"));
    }
    let receipts = state
        .scientific_authority_audit
        .stream_receipts(claim_id)
        .await?;
    let receipt_event_ids = receipts
        .iter()
        .map(|receipt| receipt.receipt.event_id)
        .collect::<std::collections::BTreeSet<_>>();
    let mut unattested = Vec::new();
    for event in &stream {
        if receipt_event_ids.contains(&event.envelope.event_id) {
            continue;
        }
        unattested.push(json!({
            "event_id": event.envelope.event_id,
            "sequence": event.envelope.sequence,
            "status": state
                .scientific_authority_audit
                .status(event.envelope.event_id)
                .await?,
        }));
    }
    Ok(Json(json!({
        "stream_id": claim_id,
        "receipts": receipts,
        "unattested_events": unattested,
    })))
}

/// Retrieve one signed event by its globally unique event identifier.
pub async fn get_scientific_event(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<serde_json::Value>> {
    let event = state
        .scientific_events
        .event_by_id(ScientificEventId(id))
        .await?
        .ok_or_else(|| ApiError::not_found("scientific event"))?;
    let event_hash = event.event_hash()?;
    let event_id = ScientificEventId(id);
    let authority_receipt = state.scientific_authority_audit.receipt(event_id).await?;
    let authority_status = state.scientific_authority_audit.status(event_id).await?;
    Ok(Json(json!({
        "event_hash": event_hash,
        "event": event,
        "authority_receipt": authority_receipt,
        "authority_status": authority_status,
    })))
}

/// Retrieve the independently persisted receipt-time authority evidence for an event.
pub async fn get_scientific_event_authority_receipt(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<serde_json::Value>> {
    let receipt = state
        .scientific_authority_audit
        .receipt(ScientificEventId(id))
        .await?
        .ok_or_else(|| ApiError::not_found("scientific authority receipt"))?;
    Ok(Json(json!({
        "receipt_hash": receipt.receipt_hash()?,
        "authority_receipt": receipt,
    })))
}
