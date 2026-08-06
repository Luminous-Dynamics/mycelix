// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Threshold scientific credential-governance API.

use axum::{
    Extension, Json,
    extract::{Path, Query, State},
    http::StatusCode,
};
use mycelix_desci_core::{
    CredentialGovernanceEventId, CredentialGovernancePayload, CredentialGovernanceProposalId,
};
use serde_json::json;
use std::sync::Arc;
use uuid::Uuid;

use crate::{
    error::{ApiError, Result},
    middleware::AuthenticatedActor,
    models::{AppendCredentialGovernanceEventRequest, CredentialGovernancePageQuery},
    state::AppState,
};

pub async fn append_credential_governance_event(
    State(state): State<Arc<AppState>>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(request): Json<AppendCredentialGovernanceEventRequest>,
) -> Result<(StatusCode, Json<serde_json::Value>)> {
    if actor.subject() != request.event.envelope.actor.as_str() {
        return Err(ApiError::forbidden(
            "JWT subject must exactly match the signed governance event actor",
        ));
    }
    match &request.event.envelope.payload {
        CredentialGovernancePayload::PolicyInitialized { .. } => {
            return Err(ApiError::invalid_request(
                "credential governance initialization is an offline bootstrap operation",
            ));
        }
        CredentialGovernancePayload::ProposalExecuted { .. } => {
            return Err(ApiError::invalid_request(
                "proposal execution must use the dedicated execute endpoint",
            ));
        }
        _ => {}
    }
    let receipt = state
        .credential_governance
        .append(request.expected_sequence, request.event)
        .await?;
    Ok((
        StatusCode::CREATED,
        Json(json!({
            "receipt": receipt,
            "summary": state.credential_governance.summary().await,
        })),
    ))
}

pub async fn execute_credential_governance_proposal(
    State(state): State<Arc<AppState>>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(request): Json<AppendCredentialGovernanceEventRequest>,
) -> Result<(StatusCode, Json<serde_json::Value>)> {
    if actor.subject() != request.event.envelope.actor.as_str() {
        return Err(ApiError::forbidden(
            "JWT subject must exactly match the signed governance execution actor",
        ));
    }
    if !matches!(
        &request.event.envelope.payload,
        CredentialGovernancePayload::ProposalExecuted { .. }
    ) {
        return Err(ApiError::invalid_request(
            "execute endpoint requires proposal_executed payload",
        ));
    }
    let receipt = state
        .credential_governance
        .execute(request.expected_sequence, request.event)
        .await?;
    Ok((
        StatusCode::CREATED,
        Json(json!({
            "receipt": receipt,
            "summary": state.credential_governance.summary().await,
        })),
    ))
}

pub async fn get_credential_governance(
    State(state): State<Arc<AppState>>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    Ok(Json(json!({
        "summary": state.credential_governance.summary().await,
        "projection": state.credential_governance.projection().await,
    })))
}

pub async fn get_credential_governance_events(
    State(state): State<Arc<AppState>>,
    Query(query): Query<CredentialGovernancePageQuery>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let events = state.credential_governance.events().await;
    let total_events = events.len();
    let from_sequence_value = query.from_sequence.unwrap_or(0);
    let from_sequence = usize::try_from(from_sequence_value).map_err(|_| {
        ApiError::invalid_request("from_sequence exceeds this server's addressable range")
    })?;
    let limit = query.limit.unwrap_or(100).clamp(1, 500);
    let mut page = Vec::new();
    for recorded in events.into_iter().skip(from_sequence).take(limit) {
        page.push(json!({
            "event_hash": recorded.event.event_hash()?,
            "record_hash": recorded.record_hash()?,
            "recorded_event": recorded,
        }));
    }
    let next_sequence = from_sequence.saturating_add(page.len());
    Ok(Json(json!({
        "from_sequence": from_sequence_value,
        "limit": limit,
        "next_sequence": next_sequence,
        "has_more": next_sequence < total_events,
        "total_events": total_events,
        "events": page,
    })))
}

pub async fn get_credential_governance_proposal(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let proposal = state
        .credential_governance
        .proposal(CredentialGovernanceProposalId(id))
        .await
        .ok_or_else(|| ApiError::not_found("credential governance proposal"))?;
    let now = chrono::Utc::now();
    let registry_head = state.scientific_credentials.projection().await.head_hash;
    let effective_status = proposal.effective_status(&now, registry_head);
    let approval_status = state
        .credential_governance
        .approval_status(CredentialGovernanceProposalId(id), now)
        .await?;
    Ok(Json(json!({
        "proposal": proposal,
        "effective_status": effective_status,
        "approval_status": approval_status,
    })))
}

pub async fn get_credential_transparency_checkpoint_candidate(
    State(state): State<Arc<AppState>>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let checkpoint = state.credential_governance.checkpoint_candidate().await?;
    let checkpoint_hash = checkpoint.checkpoint_hash();
    Ok(Json(json!({
        "checkpoint": checkpoint,
        "checkpoint_hash": checkpoint_hash,
        "instructions": "publish this exact checkpoint first; governed external witnesses sign checkpoint_hash and append transparency_checkpoint_witnessed events",
    })))
}

pub async fn get_credential_governance_event(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let recorded = state
        .credential_governance
        .event(CredentialGovernanceEventId(id))
        .await
        .ok_or_else(|| ApiError::not_found("credential governance event"))?;
    Ok(Json(json!({
        "event_hash": recorded.event.event_hash()?,
        "record_hash": recorded.record_hash()?,
        "recorded_event": recorded,
    })))
}

pub async fn record_checkpoint_mirror_observation(
    State(state): State<Arc<AppState>>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(observation): Json<mycelix_desci_core::SignedCheckpointMirrorObservation>,
) -> Result<(StatusCode, Json<serde_json::Value>)> {
    state.synchronize_authority_state().await?;
    if actor.subject() != observation.actor.as_str() {
        return Err(ApiError::forbidden(
            "JWT subject must exactly match the signed checkpoint mirror actor",
        ));
    }
    let store = state.postgres_authority_backend.as_ref().ok_or_else(|| {
        ApiError::not_implemented(
            "checkpoint mirror persistence requires DESCI_SCIENTIFIC_EVENT_BACKEND=postgres",
        )
    })?;
    let projection = state.credential_governance.projection().await;
    let record = store
        .record_checkpoint_mirror_observation(observation, &projection)
        .await?;
    Ok((
        StatusCode::CREATED,
        Json(json!({
            "observation_hash": record.observation_hash,
            "accepted_at": record.accepted_at,
            "publication": "queued_in_transactional_outbox",
        })),
    ))
}

pub async fn get_checkpoint_mirror_observations(
    State(state): State<Arc<AppState>>,
    Path(checkpoint_hash): Path<String>,
) -> Result<Json<serde_json::Value>> {
    state.synchronize_authority_state().await?;
    let decoded = hex::decode(checkpoint_hash.trim()).map_err(|_| {
        ApiError::invalid_request("checkpoint hash must be 64 hexadecimal characters")
    })?;
    let bytes: [u8; 32] = decoded.try_into().map_err(|_| {
        ApiError::invalid_request("checkpoint hash must be 64 hexadecimal characters")
    })?;
    let checkpoint_hash = mycelix_desci_core::ContentHash(bytes);
    let store = state.postgres_authority_backend.as_ref().ok_or_else(|| {
        ApiError::not_implemented(
            "checkpoint mirror queries require DESCI_SCIENTIFIC_EVENT_BACKEND=postgres",
        )
    })?;
    let projection = state.credential_governance.projection().await;
    let observations = store
        .checkpoint_mirror_observations(checkpoint_hash)
        .await?;
    let mut rendered = Vec::with_capacity(observations.len());
    for record in observations {
        let verification = record.observation.verify_against_governance(&projection);
        rendered.push(json!({
            "observation_hash": record.observation_hash,
            "accepted_at": record.accepted_at,
            "currently_valid": verification.is_ok(),
            "verification_error": verification.err().map(|error| error.to_string()),
            "observation": record.observation,
        }));
    }
    Ok(Json(json!({
        "checkpoint_hash": checkpoint_hash,
        "observations": rendered,
    })))
}
