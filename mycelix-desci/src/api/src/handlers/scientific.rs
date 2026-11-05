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
use mycelix_desci_core::{
    AuthorityAttestationStatus, ClaimId, ClaimProjection, ScientificEventId,
    ScientificEventPayload,
};
use serde_json::json;
use std::{collections::HashMap, sync::Arc};
use uuid::Uuid;

use crate::{
    error::{ApiError, Result},
    middleware::AuthenticatedActor,
    models::{
        AppendScientificEventRequest, ScientificClaimAsKnownAtQuery, ScientificEventPageQuery,
    },
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

/// Reconstruct the maximal contiguous claim-event prefix that had a committed
/// server authority receipt by the requested transaction time.
///
/// This is deliberately not a source-valid-time or model-availability query:
/// receipt time proves when the canonical record was committed under the
/// authority trust model, not when a source first existed or a downstream
/// index/model incorporated it.
pub async fn get_scientific_claim_as_known_at(
    State(state): State<Arc<AppState>>,
    Path(id): Path<Uuid>,
    Query(query): Query<ScientificClaimAsKnownAtQuery>,
) -> Result<Json<serde_json::Value>> {
    let claim_id = ClaimId(id);
    let stream = state.scientific_events.stream(claim_id).await?;
    if stream.is_empty() {
        return Err(ApiError::not_found("canonical scientific claim"));
    }

    let mut events_by_id = HashMap::with_capacity(stream.len());
    for (index, event) in stream.iter().enumerate() {
        if event.envelope.stream_id != claim_id || event.envelope.sequence != index as u64 {
            return Err(ApiError::internal_error(
                "canonical event stream has a non-contiguous sequence or mismatched stream id",
            ));
        }
        if events_by_id
            .insert(event.envelope.event_id, event)
            .is_some()
        {
            return Err(ApiError::internal_error(
                "canonical event stream contains a duplicate event id",
            ));
        }
    }

    let receipts = state
        .scientific_authority_audit
        .stream_receipts(claim_id)
        .await?;
    let mut receipts_by_event_id = HashMap::with_capacity(receipts.len());

    for signed_receipt in &receipts {
        let receipt = &signed_receipt.receipt;
        let Some(event) = events_by_id.get(&receipt.event_id) else {
            return Err(ApiError::internal_error(
                "authority receipt references an event absent from the claim stream",
            ));
        };
        if receipt.stream_id != claim_id
            || receipt.sequence != event.envelope.sequence
            || receipt.event_id != event.envelope.event_id
            || receipt.event_hash != event.event_hash()?
        {
            return Err(ApiError::internal_error(
                "authority receipt does not bind the corresponding canonical event",
            ));
        }
        if receipts_by_event_id
            .insert(receipt.event_id, signed_receipt)
            .is_some()
        {
            return Err(ApiError::internal_error(
                "authority receipt chain contains a duplicate event receipt",
            ));
        }
    }

    let mut receipt_times = Vec::with_capacity(stream.len());
    let mut receipt_statuses = Vec::with_capacity(stream.len());
    for event in &stream {
        let status = state
            .scientific_authority_audit
            .status(event.envelope.event_id)
            .await?;
        match (receipts_by_event_id.get(&event.envelope.event_id), status) {
            (Some(signed_receipt), AuthorityAttestationStatus::ReceiptAttested) => {
                receipt_times.push(Some(signed_receipt.receipt.received_at.clone()));
                receipt_statuses.push("receipt_attested".to_string());
            }
            (None, AuthorityAttestationStatus::ReceiptAttested) => {
                return Err(ApiError::internal_error(
                    "event is classified receipt-attested but its committed receipt is unavailable",
                ));
            }
            (Some(_), _) => {
                return Err(ApiError::internal_error(
                    "event has a stored authority receipt but is not classified receipt-attested",
                ));
            }
            (None, other) => {
                receipt_times.push(None);
                receipt_statuses.push(format!("{other:?}"));
            }
        }
    }

    // A wall-clock as-of cutoff is not reconstructable when the trusted
    // receipt sequence moves backward in time. Preserve the evidence and
    // fail closed rather than silently sorting by timestamp.
    if !receipt_times_are_monotonic(&receipt_times) {
        return Err(ApiError::internal_error(
            "authority receipt times move backward relative to stream sequence; transaction-time projection is indeterminate",
        ));
    }

    let included_count = transaction_time_prefix_len(&receipt_times, &query.known_at);
    let included_events: Vec<_> = stream.iter().take(included_count).cloned().collect();
    let projection = if included_events.is_empty() {
        None
    } else {
        Some(ClaimProjection::rebuild(&included_events)?)
    };
    let assessment = projection.as_ref().map(ClaimProjection::assessment);

    let last_included = match included_count.checked_sub(1) {
        Some(index) => {
            let event = &stream[index];
            let signed_receipt = receipts_by_event_id
                .get(&event.envelope.event_id)
                .expect("included event has a committed receipt");
            Some(json!({
                "sequence": event.envelope.sequence,
                "event_id": event.envelope.event_id,
                "event_hash": event.event_hash()?,
                "receipt_hash": signed_receipt.receipt_hash()?,
                "received_at": signed_receipt.receipt.received_at,
            }))
        }
        None => None,
    };

    let blocked_event = if included_count < stream.len() {
        let event = &stream[included_count];
        match receipt_times[included_count].as_ref() {
            Some(received_at) => json!({
                "event_id": event.envelope.event_id,
                "sequence": event.envelope.sequence,
                "status": receipt_statuses[included_count].as_str(),
                "reason": "authority receipt is later than the requested known_at cutoff",
                "received_at": received_at,
            }),
            None => json!({
                "event_id": event.envelope.event_id,
                "sequence": event.envelope.sequence,
                "status": receipt_statuses[included_count],
                "reason": "event has no committed authority receipt; its transaction time is unknown",
            }),
        }
    } else {
        serde_json::Value::Null
    };

    Ok(Json(json!({
        "stream_id": claim_id,
        "as_known_at": {
            "known_at": query.known_at,
            "coordinate": "server_authority_receipt_received_at",
            "semantics": "maximal contiguous event prefix with committed receipts received no later than known_at",
            "does_not_establish": [
                "source acquisition or first availability time",
                "source valid time or proposition truth time",
                "when search indexes, embeddings, caches, or models incorporated the event"
            ],
        },
        "projection": projection,
        "assessment": assessment,
        "included_event_count": included_count,
        "complete": included_count == stream.len(),
        "last_included": last_included,
        "blocked_event": blocked_event,
    })))
}

fn transaction_time_prefix_len(
    receipt_times: &[Option<chrono::DateTime<chrono::Utc>>],
    known_at: &chrono::DateTime<chrono::Utc>,
) -> usize {
    let mut included = 0;
    for received_at in receipt_times {
        match received_at {
            Some(received_at) if received_at <= known_at => included += 1,
            _ => break,
        }
    }
    included
}

fn receipt_times_are_monotonic(
    receipt_times: &[Option<chrono::DateTime<chrono::Utc>>],
) -> bool {
    let mut previous: Option<&chrono::DateTime<chrono::Utc>> = None;
    for received_at in receipt_times {
        match received_at {
            Some(received_at) => {
                if previous.is_some_and(|previous| received_at < previous) {
                    return false;
                }
                previous = Some(received_at);
            }
            None => previous = None,
        }
    }
    true
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


#[cfg(test)]
mod as_known_at_projection_tests {
    use super::*;
    use chrono::{TimeZone, Utc};

    fn instant(minute: u32) -> chrono::DateTime<Utc> {
        Utc.with_ymd_and_hms(2026, 11, 5, 17, minute, 0)
            .single()
            .expect("valid fixture time")
    }

    #[test]
    fn transaction_cutoff_includes_only_the_receipt_time_prefix() {
        let times = vec![Some(instant(0)), Some(instant(1)), Some(instant(2))];
        assert_eq!(transaction_time_prefix_len(&times, &instant(1)), 2);
    }

    #[test]
    fn missing_receipt_stops_projection_even_when_later_event_has_receipt() {
        let times = vec![Some(instant(0)), None, Some(instant(1))];
        assert_eq!(transaction_time_prefix_len(&times, &instant(2)), 1);
    }

    #[test]
    fn timestamp_cutoff_cannot_skip_an_earlier_stream_sequence() {
        let times = vec![Some(instant(2)), Some(instant(1))];
        assert_eq!(transaction_time_prefix_len(&times, &instant(1)), 0);
        assert!(!receipt_times_are_monotonic(&times));
    }

    #[test]
    fn equal_receipt_timestamps_remain_sequence_ordered() {
        let times = vec![Some(instant(1)), Some(instant(1))];
        assert_eq!(transaction_time_prefix_len(&times, &instant(1)), 2);
        assert!(receipt_times_are_monotonic(&times));
    }

    #[test]
    fn unattested_prefix_does_not_reset_a_complete_as_of_projection() {
        let times = vec![None, Some(instant(1)), Some(instant(2))];
        assert_eq!(transaction_time_prefix_len(&times, &instant(2)), 0);
        assert!(receipt_times_are_monotonic(&times));
    }
}
