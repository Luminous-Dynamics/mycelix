use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use std::collections::BTreeSet;
use thiserror::Error;

pub const EVENT_PROVIDER_SCHEMA_VERSION: u16 = 1;
pub const MAX_ID_LEN: usize = 256;
pub const MAX_BINDING_LEN: usize = 512;
pub const MAX_EVENT_NAME_LEN: usize = 256;
pub const MAX_PAYLOAD_BYTES: usize = 64 * 1024;
pub const MAX_PROJECTION_ERROR_LEN: usize = 512;
pub const COMMITMENT_PREFIX: &str = "blake3-256:";

const PAYLOAD_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-EVENT-PAYLOAD\0V1\0";
const EVENT_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-DURABLE-EVENT\0V1\0";

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum EventProviderError {
    #[error("{0}")]
    Violation(String),
    #[error("json canonicalization failed: {0}")]
    Json(String),
}

type EventResult<T> = Result<T, EventProviderError>;

fn violation(message: impl Into<String>) -> EventProviderError {
    EventProviderError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> EventResult<()> {
    if value.trim().is_empty() || value.len() > max_len {
        return Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )));
    }
    Ok(())
}

fn require_did(label: &str, value: &str) -> EventResult<()> {
    require_opaque(label, value, MAX_ID_LEN)?;
    if !value.starts_with("did:") {
        return Err(violation(format!("{label} must be a DID")));
    }
    Ok(())
}

fn require_commitment(label: &str, value: &str) -> EventResult<()> {
    let digest = value
        .strip_prefix(COMMITMENT_PREFIX)
        .ok_or_else(|| violation(format!("{label} must use {COMMITMENT_PREFIX}<hex>")))?;
    if digest.len() != 64
        || !digest
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(violation(format!(
            "{label} must contain exactly 64 lowercase hexadecimal BLAKE3-256 digits"
        )));
    }
    Ok(())
}

/// Authority material that a future DHT provider must verify before accepting
/// the durable event. In E0, `claim_binding_commitment` remains deliberately
/// opaque: 003B4 + CR1 must qualify and be integrated before this crate may
/// claim that the commitment is cryptographically/runtime verified.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EventAuthorityBinding {
    pub operation_id: String,
    pub action_id: String,
    pub proposal_id: String,
    pub claim_binding_commitment: String,
    pub action_commitment: String,
    pub publisher_did: String,
}

impl EventAuthorityBinding {
    pub fn validate(&self) -> EventResult<()> {
        require_opaque("operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("action_id", &self.action_id, MAX_ID_LEN)?;
        require_opaque("proposal_id", &self.proposal_id, MAX_ID_LEN)?;
        require_opaque(
            "claim_binding_commitment",
            &self.claim_binding_commitment,
            MAX_BINDING_LEN,
        )?;
        require_opaque(
            "action_commitment",
            &self.action_commitment,
            MAX_BINDING_LEN,
        )?;
        require_did("publisher_did", &self.publisher_did)?;
        Ok(())
    }
}

/// Durable constitutional event contract.
///
/// `committed_at_unix_ms` is intentionally NOT part of event identity. A retry
/// of the same constitutional action at a later wall-clock time must resolve to
/// the already-existing event instead of becoming a second constitutional fact.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DurableConstitutionalEvent {
    pub schema_version: u16,
    pub authority: EventAuthorityBinding,
    pub event_name: String,
    /// Deterministic compact JSON with recursively sorted object keys.
    pub canonical_payload: String,
    /// Domain-separated, self-describing BLAKE3-256 payload commitment.
    pub payload_commitment: String,
    /// Domain-separated, self-describing BLAKE3-256 event commitment.
    pub event_commitment: String,
    pub committed_at_unix_ms: u64,
}

impl DurableConstitutionalEvent {
    pub fn new(
        authority: EventAuthorityBinding,
        event_name: impl Into<String>,
        payload: &Value,
        committed_at_unix_ms: u64,
    ) -> EventResult<Self> {
        authority.validate()?;
        let event_name = event_name.into();
        require_opaque("event_name", &event_name, MAX_EVENT_NAME_LEN)?;

        let canonical_payload = canonical_json(payload)?;
        if canonical_payload.len() > MAX_PAYLOAD_BYTES {
            return Err(violation(format!(
                "canonical payload must be <= {MAX_PAYLOAD_BYTES} bytes"
            )));
        }

        let payload_commitment = payload_commitment(&canonical_payload);
        let event_commitment = event_commitment(
            EVENT_PROVIDER_SCHEMA_VERSION,
            &authority,
            &event_name,
            &payload_commitment,
        );

        Ok(Self {
            schema_version: EVENT_PROVIDER_SCHEMA_VERSION,
            authority,
            event_name,
            canonical_payload,
            payload_commitment,
            event_commitment,
            committed_at_unix_ms,
        })
    }

    pub fn validate(&self) -> EventResult<()> {
        if self.schema_version != EVENT_PROVIDER_SCHEMA_VERSION {
            return Err(violation(format!(
                "unsupported event provider schema version {}",
                self.schema_version
            )));
        }
        self.authority.validate()?;
        require_opaque("event_name", &self.event_name, MAX_EVENT_NAME_LEN)?;
        require_commitment("payload_commitment", &self.payload_commitment)?;
        require_commitment("event_commitment", &self.event_commitment)?;
        if self.canonical_payload.len() > MAX_PAYLOAD_BYTES {
            return Err(violation(format!(
                "canonical payload must be <= {MAX_PAYLOAD_BYTES} bytes"
            )));
        }

        let parsed: Value = serde_json::from_str(&self.canonical_payload)
            .map_err(|e| EventProviderError::Json(e.to_string()))?;
        let recanonicalized = canonical_json(&parsed)?;
        if recanonicalized != self.canonical_payload {
            return Err(violation(
                "canonical_payload is valid JSON but not in canonical form",
            ));
        }

        let expected_payload = payload_commitment(&self.canonical_payload);
        if self.payload_commitment != expected_payload {
            return Err(violation(
                "payload_commitment does not match canonical_payload",
            ));
        }

        let expected_event = event_commitment(
            self.schema_version,
            &self.authority,
            &self.event_name,
            &self.payload_commitment,
        );
        if self.event_commitment != expected_event {
            return Err(violation(
                "event_commitment does not match durable event semantics",
            ));
        }

        Ok(())
    }

    /// Provider key. Constitutional event idempotency is scoped to the exact
    /// action identity, not wall-clock time or generated record hashes.
    pub fn provider_key(&self) -> &str {
        &self.authority.action_id
    }

    pub fn semantic_commitment(&self) -> &str {
        &self.event_commitment
    }
}

/// Closed publish decision for an action-keyed event store.
///
/// There is intentionally no Overwrite/Replace case. Conflicting semantics
/// under one action identity are an integrity conflict, not last-write-wins.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PublishDecision {
    Created {
        event: DurableConstitutionalEvent,
    },
    ExistingSame {
        action_id: String,
        event_commitment: String,
    },
    IntegrityConflict {
        action_id: String,
        existing_commitment: String,
        candidate_commitment: String,
    },
}

pub fn decide_publish(
    existing: Option<&DurableConstitutionalEvent>,
    candidate: DurableConstitutionalEvent,
) -> EventResult<PublishDecision> {
    candidate.validate()?;

    let Some(existing) = existing else {
        return Ok(PublishDecision::Created { event: candidate });
    };

    existing.validate()?;
    if existing.provider_key() != candidate.provider_key() {
        return Err(violation(
            "existing event provider key does not match candidate action_id",
        ));
    }

    if existing.semantic_commitment() == candidate.semantic_commitment() {
        return Ok(PublishDecision::ExistingSame {
            action_id: candidate.authority.action_id,
            event_commitment: candidate.event_commitment,
        });
    }

    Ok(PublishDecision::IntegrityConflict {
        action_id: candidate.authority.action_id,
        existing_commitment: existing.event_commitment.clone(),
        candidate_commitment: candidate.event_commitment,
    })
}

/// Signal delivery is a projection of durable event truth, not the truth itself.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProjectionOutcome {
    Delivered,
    Failed { error_class: String },
    UnknownOutcome,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SignalProjectionAttempt {
    pub projection_attempt_id: String,
    pub operation_id: String,
    pub action_id: String,
    pub durable_event_commitment: String,
    pub attempt_ordinal: u32,
    pub attempted_at_unix_ms: u64,
    pub outcome: ProjectionOutcome,
}

impl SignalProjectionAttempt {
    pub fn validate_against(&self, event: &DurableConstitutionalEvent) -> EventResult<()> {
        event.validate()?;
        require_opaque(
            "projection_attempt_id",
            &self.projection_attempt_id,
            MAX_ID_LEN,
        )?;
        require_opaque("projection operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque("projection action_id", &self.action_id, MAX_ID_LEN)?;
        require_commitment(
            "durable_event_commitment",
            &self.durable_event_commitment,
        )?;
        if self.attempt_ordinal == 0 {
            return Err(violation("projection attempt_ordinal must be >= 1"));
        }
        if self.operation_id != event.authority.operation_id
            || self.action_id != event.authority.action_id
            || self.durable_event_commitment != event.event_commitment
        {
            return Err(violation(
                "signal projection is not bound to the durable constitutional event",
            ));
        }
        if let ProjectionOutcome::Failed { error_class } = &self.outcome {
            require_opaque(
                "projection failure error_class",
                error_class,
                MAX_PROJECTION_ERROR_LEN,
            )?;
        }
        Ok(())
    }
}

/// Validate projection diagnostics as an append-only per-event history.
///
/// Projection delivery remains non-authoritative for constitutional completion,
/// but diagnostic evidence must itself be coherent: attempt IDs are unique,
/// ordinals are contiguous from one, and recorded attempt time cannot move
/// backwards through the history.
pub fn validate_projection_history(
    event: &DurableConstitutionalEvent,
    projections: &[SignalProjectionAttempt],
) -> EventResult<()> {
    event.validate()?;
    let mut seen_attempt_ids = BTreeSet::new();
    let mut expected_ordinal = 1u32;
    let mut previous_time: Option<u64> = None;

    for projection in projections {
        projection.validate_against(event)?;

        if !seen_attempt_ids.insert(projection.projection_attempt_id.as_str()) {
            return Err(violation("duplicate projection_attempt_id in history"));
        }
        if projection.attempt_ordinal != expected_ordinal {
            return Err(violation(format!(
                "projection attempt ordinals must be contiguous from 1: expected {expected_ordinal}, got {}",
                projection.attempt_ordinal
            )));
        }
        if let Some(previous) = previous_time {
            if projection.attempted_at_unix_ms < previous {
                return Err(violation(
                    "projection attempted_at_unix_ms must be non-decreasing",
                ));
            }
        }

        expected_ordinal = expected_ordinal
            .checked_add(1)
            .ok_or_else(|| violation("projection attempt ordinal overflow"))?;
        previous_time = Some(projection.attempted_at_unix_ms);
    }

    Ok(())
}

/// Constitutional completion is defined by valid durable event truth only.
/// Projection history is validated for binding and internal coherence, but its
/// delivery outcomes do not create, revoke, or rewrite the constitutional event.
pub fn constitutional_event_complete(
    event: &DurableConstitutionalEvent,
    projections: &[SignalProjectionAttempt],
) -> EventResult<bool> {
    validate_projection_history(event, projections)?;
    Ok(true)
}

/// Deterministic compact JSON. Object keys are recursively sorted; array order
/// is preserved because array ordering is semantic JSON content.
pub fn canonical_json(value: &Value) -> EventResult<String> {
    let normalized = normalize_json(value);
    serde_json::to_string(&normalized).map_err(|e| EventProviderError::Json(e.to_string()))
}

fn normalize_json(value: &Value) -> Value {
    match value {
        Value::Object(map) => {
            let mut keys: Vec<&String> = map.keys().collect();
            keys.sort_unstable();
            let mut normalized = Map::new();
            for key in keys {
                let child = map
                    .get(key)
                    .expect("key was collected from the same JSON object");
                normalized.insert(key.clone(), normalize_json(child));
            }
            Value::Object(normalized)
        }
        Value::Array(values) => Value::Array(values.iter().map(normalize_json).collect()),
        other => other.clone(),
    }
}

pub fn payload_commitment(canonical_payload: &str) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(PAYLOAD_DOMAIN);
    push_bytes(&mut hasher, canonical_payload.as_bytes());
    tagged_hash(hasher.finalize())
}

fn event_commitment(
    schema_version: u16,
    authority: &EventAuthorityBinding,
    event_name: &str,
    payload_commitment: &str,
) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(EVENT_DOMAIN);
    hasher.update(&schema_version.to_be_bytes());
    push_str(&mut hasher, &authority.operation_id);
    push_str(&mut hasher, &authority.action_id);
    push_str(&mut hasher, &authority.proposal_id);
    push_str(&mut hasher, &authority.claim_binding_commitment);
    push_str(&mut hasher, &authority.action_commitment);
    push_str(&mut hasher, &authority.publisher_did);
    push_str(&mut hasher, event_name);
    push_str(&mut hasher, payload_commitment);
    tagged_hash(hasher.finalize())
}

fn tagged_hash(hash: blake3::Hash) -> String {
    format!("{COMMITMENT_PREFIX}{}", hash.to_hex())
}

fn push_str(hasher: &mut blake3::Hasher, value: &str) {
    push_bytes(hasher, value.as_bytes());
}

fn push_bytes(hasher: &mut blake3::Hasher, value: &[u8]) {
    hasher.update(&(value.len() as u64).to_be_bytes());
    hasher.update(value);
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn authority() -> EventAuthorityBinding {
        EventAuthorityBinding {
            operation_id: "operation-001".into(),
            action_id: "action-002".into(),
            proposal_id: "proposal-003".into(),
            claim_binding_commitment: "claim-binding:abc123".into(),
            action_commitment: "action-commitment:def456".into(),
            publisher_did: "did:mycelix:test-publisher".into(),
        }
    }

    fn event_at(timestamp: u64) -> DurableConstitutionalEvent {
        DurableConstitutionalEvent::new(
            authority(),
            "ProposalExecuted",
            &json!({
                "z": 2,
                "a": {"second": true, "first": 1},
                "items": [3, 2, 1]
            }),
            timestamp,
        )
        .unwrap()
    }

    fn projection(
        event: &DurableConstitutionalEvent,
        id: &str,
        ordinal: u32,
        time: u64,
        outcome: ProjectionOutcome,
    ) -> SignalProjectionAttempt {
        SignalProjectionAttempt {
            projection_attempt_id: id.into(),
            operation_id: event.authority.operation_id.clone(),
            action_id: event.authority.action_id.clone(),
            durable_event_commitment: event.event_commitment.clone(),
            attempt_ordinal: ordinal,
            attempted_at_unix_ms: time,
            outcome,
        }
    }

    #[test]
    fn canonical_json_sorts_object_keys_recursively() {
        let left = json!({"z": 2, "a": {"second": true, "first": 1}});
        let right = json!({"a": {"first": 1, "second": true}, "z": 2});
        assert_eq!(canonical_json(&left).unwrap(), canonical_json(&right).unwrap());
    }

    #[test]
    fn canonical_json_preserves_array_order() {
        let left = json!([1, 2, 3]);
        let right = json!([3, 2, 1]);
        assert_ne!(canonical_json(&left).unwrap(), canonical_json(&right).unwrap());
    }

    #[test]
    fn commitments_are_self_describing_blake3_256() {
        let event = event_at(1_000);
        for commitment in [&event.payload_commitment, &event.event_commitment] {
            assert!(commitment.starts_with(COMMITMENT_PREFIX));
            assert_eq!(commitment.len(), COMMITMENT_PREFIX.len() + 64);
            require_commitment("test commitment", commitment).unwrap();
        }
    }

    #[test]
    fn wall_clock_time_does_not_define_event_identity() {
        let first = event_at(1_000);
        let retried_later = event_at(9_000);
        assert_ne!(first.committed_at_unix_ms, retried_later.committed_at_unix_ms);
        assert_eq!(first.event_commitment, retried_later.event_commitment);
    }

    #[test]
    fn first_publish_creates_durable_event() {
        let candidate = event_at(1_000);
        match decide_publish(None, candidate.clone()).unwrap() {
            PublishDecision::Created { event } => assert_eq!(event, candidate),
            other => panic!("expected Created, got {other:?}"),
        }
    }

    #[test]
    fn duplicate_same_action_and_semantics_returns_existing_same() {
        let existing = event_at(1_000);
        let retry = event_at(9_000);
        match decide_publish(Some(&existing), retry).unwrap() {
            PublishDecision::ExistingSame {
                action_id,
                event_commitment,
            } => {
                assert_eq!(action_id, existing.authority.action_id);
                assert_eq!(event_commitment, existing.event_commitment);
            }
            other => panic!("expected ExistingSame, got {other:?}"),
        }
    }

    #[test]
    fn same_action_id_with_different_payload_is_integrity_conflict() {
        let existing = event_at(1_000);
        let conflict = DurableConstitutionalEvent::new(
            authority(),
            "ProposalExecuted",
            &json!({"different": true}),
            2_000,
        )
        .unwrap();

        match decide_publish(Some(&existing), conflict).unwrap() {
            PublishDecision::IntegrityConflict {
                action_id,
                existing_commitment,
                candidate_commitment,
            } => {
                assert_eq!(action_id, existing.authority.action_id);
                assert_eq!(existing_commitment, existing.event_commitment);
                assert_ne!(candidate_commitment, existing.event_commitment);
            }
            other => panic!("expected IntegrityConflict, got {other:?}"),
        }
    }

    #[test]
    fn same_action_id_with_different_claim_binding_is_integrity_conflict() {
        let existing = event_at(1_000);
        let mut changed_authority = authority();
        changed_authority.claim_binding_commitment = "claim-binding:OTHER".into();
        let conflict = DurableConstitutionalEvent::new(
            changed_authority,
            "ProposalExecuted",
            &json!({
                "z": 2,
                "a": {"second": true, "first": 1},
                "items": [3, 2, 1]
            }),
            2_000,
        )
        .unwrap();
        assert!(matches!(
            decide_publish(Some(&existing), conflict).unwrap(),
            PublishDecision::IntegrityConflict { .. }
        ));
    }

    #[test]
    fn same_action_id_with_different_publisher_is_integrity_conflict() {
        let existing = event_at(1_000);
        let mut changed_authority = authority();
        changed_authority.publisher_did = "did:mycelix:someone-else".into();
        let conflict = DurableConstitutionalEvent::new(
            changed_authority,
            "ProposalExecuted",
            &json!({
                "z": 2,
                "a": {"second": true, "first": 1},
                "items": [3, 2, 1]
            }),
            2_000,
        )
        .unwrap();
        assert!(matches!(
            decide_publish(Some(&existing), conflict).unwrap(),
            PublishDecision::IntegrityConflict { .. }
        ));
    }

    #[test]
    fn malformed_payload_commitment_is_rejected() {
        let mut event = event_at(1_000);
        event.payload_commitment = "tampered".into();
        assert!(event.validate().is_err());
    }

    #[test]
    fn malformed_event_commitment_is_rejected() {
        let mut event = event_at(1_000);
        event.event_commitment = "tampered".into();
        assert!(event.validate().is_err());
    }

    #[test]
    fn uppercase_commitment_hex_is_rejected() {
        let mut event = event_at(1_000);
        event.event_commitment = event.event_commitment.to_ascii_uppercase();
        assert!(event.validate().is_err());
    }

    #[test]
    fn noncanonical_payload_text_is_rejected() {
        let mut event = event_at(1_000);
        event.canonical_payload = "{ \"z\": 2, \"a\": 1 }".into();
        event.payload_commitment = payload_commitment(&event.canonical_payload);
        event.event_commitment = event_commitment(
            event.schema_version,
            &event.authority,
            &event.event_name,
            &event.payload_commitment,
        );
        assert!(event.validate().is_err());
    }

    #[test]
    fn authority_binding_requires_did_publisher() {
        let mut binding = authority();
        binding.publisher_did = "alice".into();
        assert!(binding.validate().is_err());
    }

    #[test]
    fn failed_signal_projection_does_not_revoke_constitutional_completion() {
        let event = event_at(1_000);
        let failed = projection(
            &event,
            "projection-1",
            1,
            1_100,
            ProjectionOutcome::Failed {
                error_class: "subscriber-disconnected".into(),
            },
        );
        assert!(constitutional_event_complete(&event, &[failed]).unwrap());
    }

    #[test]
    fn signal_projection_can_be_retried_without_new_constitutional_event() {
        let event = event_at(1_000);
        let first = projection(
            &event,
            "projection-1",
            1,
            1_100,
            ProjectionOutcome::Failed {
                error_class: "subscriber-disconnected".into(),
            },
        );
        let second = projection(
            &event,
            "projection-2",
            2,
            2_100,
            ProjectionOutcome::Delivered,
        );
        assert!(constitutional_event_complete(&event, &[first, second]).unwrap());
    }

    #[test]
    fn duplicate_projection_attempt_id_is_rejected() {
        let event = event_at(1_000);
        let first = projection(
            &event,
            "projection-1",
            1,
            1_100,
            ProjectionOutcome::UnknownOutcome,
        );
        let duplicate_id = projection(
            &event,
            "projection-1",
            2,
            1_200,
            ProjectionOutcome::Delivered,
        );
        assert!(validate_projection_history(&event, &[first, duplicate_id]).is_err());
    }

    #[test]
    fn projection_ordinal_gap_is_rejected() {
        let event = event_at(1_000);
        let first = projection(
            &event,
            "projection-1",
            1,
            1_100,
            ProjectionOutcome::UnknownOutcome,
        );
        let gap = projection(
            &event,
            "projection-3",
            3,
            1_300,
            ProjectionOutcome::Delivered,
        );
        assert!(validate_projection_history(&event, &[first, gap]).is_err());
    }

    #[test]
    fn projection_time_regression_is_rejected() {
        let event = event_at(1_000);
        let first = projection(
            &event,
            "projection-1",
            1,
            2_000,
            ProjectionOutcome::UnknownOutcome,
        );
        let regressed = projection(
            &event,
            "projection-2",
            2,
            1_900,
            ProjectionOutcome::Delivered,
        );
        assert!(validate_projection_history(&event, &[first, regressed]).is_err());
    }

    #[test]
    fn projection_for_another_event_is_rejected() {
        let event = event_at(1_000);
        let other = SignalProjectionAttempt {
            action_id: "another-action".into(),
            ..projection(
                &event,
                "projection-1",
                1,
                1_100,
                ProjectionOutcome::Delivered,
            )
        };
        assert!(other.validate_against(&event).is_err());
    }

    #[test]
    fn existing_lookup_must_be_keyed_to_candidate_action() {
        let existing = event_at(1_000);
        let mut candidate_authority = authority();
        candidate_authority.action_id = "different-action".into();
        let candidate = DurableConstitutionalEvent::new(
            candidate_authority,
            "ProposalExecuted",
            &json!({"ok": true}),
            2_000,
        )
        .unwrap();
        assert!(decide_publish(Some(&existing), candidate).is_err());
    }
}
