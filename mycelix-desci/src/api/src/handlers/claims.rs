// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Claims API handlers

use axum::{
    Extension, Json,
    extract::{Path, State},
    http::StatusCode,
};
use mycelix_desci_core::claims::{ClaimContent, DesciClaim, Provenance, Verification};
use mycelix_desci_core::storage::StorageBackend;
use std::sync::Arc;
use tracing::{info, instrument};
use uuid::Uuid;

use crate::{
    error::{ApiError, Result},
    metrics,
    middleware::AuthenticatedActor,
    models::*,
    state::AppState,
};

fn ensure_legacy_mutations_enabled(state: &AppState) -> Result<()> {
    if state.legacy_mutations_enabled {
        Ok(())
    } else {
        Err(ApiError::gone(
            "legacy mutable claim writes are disabled; submit a signed canonical event to /api/v1/scientific/events",
        ))
    }
}

/// Create a new claim
#[utoipa::path(
    post,
    path = "/api/v1/claims",
    request_body = CreateClaimRequest,
    responses(
        (status = 201, description = "Claim created successfully", body = ClaimResponse),
        (status = 400, description = "Invalid request", body = ApiError),
    ),
    tag = "claims"
)]
#[instrument(skip(state), fields(category = %req.content.category))]
pub async fn create_claim(
    State(state): State<Arc<AppState>>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(req): Json<CreateClaimRequest>,
) -> Result<(StatusCode, Json<ClaimResponse>)> {
    info!("Creating new claim");

    ensure_legacy_mutations_enabled(&state)?;

    // Convert request to core types
    let content = ClaimContent {
        dataset_hash: req.content.dataset_hash,
        description: req.content.description,
        category: req.content.category,
        keywords: req.content.keywords,
        storage_ref: req.content.storage_ref,
        reproducibility_score: req.content.reproducibility_score,
        license: req.content.license,
    };

    // Create claim
    let claim = DesciClaim::new(
        mycelix_desci_core::claims::EpistemicTier::E0,
        content,
        actor.subject().to_string(),
    );

    // Store claim
    state.storage.store(&claim).await?;

    // Add to query index
    state.query_engine.write().await.add_claim(&claim).await;

    // Track metrics
    metrics::track_claim_creation(&format!("{:?}", claim.epistemic_tier));
    metrics::track_storage_operation("store", true);

    info!(claim_id = %claim.id, "Claim created successfully");

    let response = ClaimResponse::from(&claim);
    Ok((StatusCode::CREATED, Json(response)))
}

/// Get claim by ID
#[utoipa::path(
    get,
    path = "/api/v1/claims/{id}",
    responses(
        (status = 200, description = "Claim found", body = ClaimResponse),
        (status = 404, description = "Claim not found", body = ApiError),
    ),
    params(
        ("id" = Uuid, Path, description = "Claim ID")
    ),
    tag = "claims"
)]
#[instrument(skip(state))]
pub async fn get_claim(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
) -> Result<Json<ClaimResponse>> {
    info!(claim_id = %id, "Retrieving claim");

    let claim = state
        .storage
        .retrieve(&id.to_string())
        .await
        .map_err(|_| ApiError::not_found("Claim"))?;

    Ok(Json(ClaimResponse::from(&claim)))
}

/// Add verification to claim
#[utoipa::path(
    put,
    path = "/api/v1/claims/{id}/verify",
    request_body = AddVerificationRequest,
    responses(
        (status = 200, description = "Verification added", body = ClaimResponse),
        (status = 404, description = "Claim not found", body = ApiError),
    ),
    params(
        ("id" = Uuid, Path, description = "Claim ID")
    ),
    tag = "claims"
)]
#[instrument(skip(state))]
pub async fn add_verification(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(req): Json<AddVerificationRequest>,
) -> Result<Json<ClaimResponse>> {
    info!(claim_id = %id, "Adding verification");

    ensure_legacy_mutations_enabled(&state)?;

    // Retrieve claim
    let mut claim = state
        .storage
        .retrieve(&id.to_string())
        .await
        .map_err(|_| ApiError::not_found("Claim"))?;

    // Record compatibility verification material without promoting maturity.
    // Signed, typed attestations in the scientific-event kernel replace this path.
    let verification = Verification {
        verifier: actor.subject().to_string(),
        signature: req.signature,
        timestamp: chrono::Utc::now(),
        notes: req.notes,
    };
    claim
        .record_unassessed_verification(verification)
        .map_err(|error| ApiError::invalid_request(error.to_string()))?;

    // Update storage
    state.storage.store(&claim).await?;

    // Update query index
    state.query_engine.write().await.add_claim(&claim).await;

    info!(claim_id = %id, new_tier = ?claim.epistemic_tier, "Verification added");

    Ok(Json(ClaimResponse::from(&claim)))
}

/// Add provenance to claim
#[utoipa::path(
    put,
    path = "/api/v1/claims/{id}/provenance",
    request_body = AddProvenanceRequest,
    responses(
        (status = 200, description = "Provenance added", body = ClaimResponse),
        (status = 404, description = "Claim not found", body = ApiError),
    ),
    params(
        ("id" = Uuid, Path, description = "Claim ID")
    ),
    tag = "claims"
)]
#[instrument(skip(state))]
pub async fn add_provenance(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
    Extension(actor): Extension<AuthenticatedActor>,
    Json(req): Json<AddProvenanceRequest>,
) -> Result<Json<ClaimResponse>> {
    info!(claim_id = %id, "Adding provenance");

    ensure_legacy_mutations_enabled(&state)?;

    // Retrieve claim
    let mut claim = state
        .storage
        .retrieve(&id.to_string())
        .await
        .map_err(|_| ApiError::not_found("Claim"))?;

    if claim.creator != actor.subject() {
        return Err(ApiError::forbidden(
            "only the authenticated claim creator may append legacy provenance",
        ));
    }

    // Add provenance
    let mut prov = Provenance::new(req.source, req.source_type);
    if let Some(url) = req.url {
        prov = prov.with_url(url);
    }
    claim.add_provenance(prov);

    // Update storage
    state.storage.store(&claim).await?;

    info!(claim_id = %id, "Provenance added");

    Ok(Json(ClaimResponse::from(&claim)))
}
