// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Mission-neutral maritime evidence payloads for the existing Commons bridge.
//!
//! No new DHT entry type is introduced. These payloads serialize into the existing
//! schema-versioned `CommonsEvent` / `BridgeEventEntry`. Position/navigation
//! measurements remain owned by `mycelix-position` and are referenced here by
//! opaque evidence identifiers.

use crate::CommonsEvent;
use hdi::prelude::{AgentPubKey, Timestamp};
use serde::{Deserialize, Serialize};

/// Existing bridge domain used for maritime evidence events.
///
/// This is only an event label. Publishing one of these events does not call or
/// depend on the dormant transport route/sharing/impact zomes.
pub const MARITIME_BRIDGE_DOMAIN: &str = "transport";

/// Dedicated event type inside the existing generic bridge event schema.
pub const MARITIME_EVENT_TYPE_V1: &str = "maritime_evidence_v1";
pub const MARITIME_SCHEMA_V1: u8 = 1;

const MAX_PLATFORM_ID_BYTES: usize = 256;
const MAX_EVIDENCE_BINDING_BYTES: usize = 1024;
const MAX_PAYLOAD_BYTES: usize = 60 * 1024;
const MAX_REFERENCE_COUNT: usize = 64;
const MAX_REFERENCE_BYTES: usize = 512;
/// `BridgeEventEntry.payload` is bounded by the shared bridge integrity contract.
/// Maritime validation must apply this to the serialized envelope, not merely
/// `payload_json`, because all wrapper metadata consumes the same outer budget.
const MAX_BRIDGE_EVENT_PAYLOAD_BYTES: usize = 8 * 1024;

/// Mission-neutral event classes. Weapon, target and engagement semantics are
/// intentionally absent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MaritimeEvidenceKind {
    StateObservation,
    HealthObservation,
    AssuranceTransition,
    AuthorityTransition,
    PositionEvidenceReference,
    CommunicationsState,
    LogisticsEvent,
    MaintenanceEvent,
    RecoveryEvent,
}

/// Portable evidence envelope for intermittent/store-and-forward operation.
///
/// `evidence_binding` is an opaque reference to the upstream evidence system
/// (for example HAL admission evidence or an authenticated Xenia transcript).
/// This type does not verify those systems' cryptography.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaritimeEvidenceEnvelope {
    pub schema_version: u8,
    pub platform_id: String,
    /// Monotonic software/evidence lineage generation for this platform chain.
    /// A successor may remain in the same generation or advance it, but may not
    /// regress to an older generation.
    pub generation: u64,
    pub sequence: u64,
    pub observed_at_us: u64,
    pub kind: MaritimeEvidenceKind,
    pub payload_json: String,
    pub evidence_binding: String,
    /// Opaque references into Mycelix Position measurement/estimate evidence.
    ///
    /// V1 requires strict lexical ordering with no duplicates so independently
    /// assembled reference sets have one digest representation.
    #[serde(default)]
    pub position_evidence_refs: Vec<String>,
    /// Content digest of the previous event. `None` marks a chain root.
    pub previous_event_digest: Option<String>,
}

impl MaritimeEvidenceEnvelope {
    pub fn new(
        platform_id: impl Into<String>,
        generation: u64,
        sequence: u64,
        observed_at_us: u64,
        kind: MaritimeEvidenceKind,
        payload_json: impl Into<String>,
        evidence_binding: impl Into<String>,
    ) -> Self {
        Self {
            schema_version: MARITIME_SCHEMA_V1,
            platform_id: platform_id.into(),
            generation,
            sequence,
            observed_at_us,
            kind,
            payload_json: payload_json.into(),
            evidence_binding: evidence_binding.into(),
            position_evidence_refs: Vec::new(),
            previous_event_digest: None,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != MARITIME_SCHEMA_V1 {
            return Err(format!(
                "unsupported maritime schema version {}",
                self.schema_version
            ));
        }
        if !canonical_text(&self.platform_id, MAX_PLATFORM_ID_BYTES) {
            return Err("platform_id is empty, oversized, padded, or contains control bytes".into());
        }
        if self.payload_json.len() > MAX_PAYLOAD_BYTES {
            return Err(format!("payload_json exceeds {} bytes", MAX_PAYLOAD_BYTES));
        }
        if self.payload_json.trim() != self.payload_json {
            return Err("payload_json must not contain outer whitespace".into());
        }
        serde_json::from_str::<serde_json::Value>(&self.payload_json)
            .map_err(|_| "payload_json must contain valid JSON".to_string())?;
        if !canonical_text(&self.evidence_binding, MAX_EVIDENCE_BINDING_BYTES) {
            return Err(
                "evidence_binding is empty, oversized, padded, or contains control bytes".into(),
            );
        }
        if self.position_evidence_refs.len() > MAX_REFERENCE_COUNT {
            return Err(format!(
                "too many position evidence references (max {})",
                MAX_REFERENCE_COUNT
            ));
        }
        for reference in &self.position_evidence_refs {
            if !canonical_text(reference, MAX_REFERENCE_BYTES) {
                return Err(
                    "position evidence reference is empty, oversized, padded, or contains control bytes"
                        .into(),
                );
            }
        }
        if self
            .position_evidence_refs
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err(
                "position evidence references must be strictly sorted and duplicate-free".into(),
            );
        }
        if self
            .previous_event_digest
            .as_deref()
            .is_some_and(|digest| !is_lower_hex_64(digest))
        {
            return Err(
                "previous_event_digest must be a canonical lowercase 64-character hex digest"
                    .into(),
            );
        }

        // The existing Commons bridge validates the full BridgeEventEntry payload at 8 KiB.
        // Measure the exact maritime envelope bytes that `to_commons_event` will place there;
        // otherwise this type could report `Valid` for evidence the DHT integrity zome rejects.
        let bridge_payload = serde_json::to_vec(self)
            .map_err(|e| format!("failed to serialize maritime payload for size validation: {e}"))?;
        if bridge_payload.len() > MAX_BRIDGE_EVENT_PAYLOAD_BYTES {
            return Err(format!(
                "serialized maritime envelope exceeds Commons bridge payload limit of {} bytes",
                MAX_BRIDGE_EVENT_PAYLOAD_BYTES
            ));
        }
        Ok(())
    }

    /// Domain-separated content digest for continuity/deduplication.
    ///
    /// This is an integrity identifier, not an authentication proof.
    pub fn content_digest(&self) -> Result<String, String> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-maritime-evidence-v1\0");
        hash_bytes(&mut hasher, &[self.schema_version]);
        hash_bytes(&mut hasher, self.platform_id.as_bytes());
        hash_bytes(&mut hasher, &self.generation.to_le_bytes());
        hash_bytes(&mut hasher, &self.sequence.to_le_bytes());
        hash_bytes(&mut hasher, &self.observed_at_us.to_le_bytes());
        hash_bytes(&mut hasher, kind_label(self.kind).as_bytes());
        hash_bytes(&mut hasher, self.payload_json.as_bytes());
        hash_bytes(&mut hasher, self.evidence_binding.as_bytes());
        for reference in &self.position_evidence_refs {
            hash_bytes(&mut hasher, reference.as_bytes());
        }
        match &self.previous_event_digest {
            Some(previous) => {
                hasher.update(&[1]);
                hash_bytes(&mut hasher, previous.as_bytes());
            }
            None => {
                hasher.update(&[0]);
            }
        }
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn chain_after(mut self, previous: &Self) -> Result<Self, String> {
        if self.platform_id != previous.platform_id {
            return Err("cannot chain maritime events from different platforms".into());
        }
        if self.generation < previous.generation {
            return Err(format!(
                "successor generation must be at least {}, got {}",
                previous.generation, self.generation
            ));
        }
        let expected_sequence = previous
            .sequence
            .checked_add(1)
            .ok_or_else(|| "predecessor sequence overflow".to_string())?;
        if self.sequence != expected_sequence {
            return Err(format!(
                "successor sequence must be {}, got {}",
                expected_sequence, self.sequence
            ));
        }
        self.previous_event_digest = Some(previous.content_digest()?);
        self.validate()?;
        Ok(self)
    }

    /// Wrap this payload in the Commons bridge's existing event entry.
    pub fn to_commons_event(
        &self,
        source_agent: AgentPubKey,
        created_at: Timestamp,
    ) -> Result<CommonsEvent, String> {
        self.validate()?;
        let payload = serde_json::to_string(self)
            .map_err(|e| format!("failed to serialize maritime payload: {e}"))?;
        Ok(CommonsEvent {
            schema_version: 1,
            domain: MARITIME_BRIDGE_DOMAIN.into(),
            event_type: MARITIME_EVENT_TYPE_V1.into(),
            source_agent,
            payload,
            created_at,
            related_hashes: Vec::new(),
        })
    }

    /// Decode a maritime payload from an existing generic Commons bridge event.
    pub fn from_commons_event(event: &CommonsEvent) -> Result<Self, String> {
        if event.domain != MARITIME_BRIDGE_DOMAIN {
            return Err("bridge event is not in the maritime transport domain".into());
        }
        if event.event_type != MARITIME_EVENT_TYPE_V1 {
            return Err("bridge event is not maritime_evidence_v1".into());
        }
        let envelope: Self = serde_json::from_str(&event.payload)
            .map_err(|e| format!("invalid maritime event payload: {e}"))?;
        envelope.validate()?;
        Ok(envelope)
    }
}

fn canonical_text(value: &str, max_bytes: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_bytes
        && value.trim() == value
        && !value.chars().any(char::is_control)
}

fn is_lower_hex_64(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn hash_bytes(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn kind_label(kind: MaritimeEvidenceKind) -> &'static str {
    match kind {
        MaritimeEvidenceKind::StateObservation => "state_observation",
        MaritimeEvidenceKind::HealthObservation => "health_observation",
        MaritimeEvidenceKind::AssuranceTransition => "assurance_transition",
        MaritimeEvidenceKind::AuthorityTransition => "authority_transition",
        MaritimeEvidenceKind::PositionEvidenceReference => "position_evidence_reference",
        MaritimeEvidenceKind::CommunicationsState => "communications_state",
        MaritimeEvidenceKind::LogisticsEvent => "logistics_event",
        MaritimeEvidenceKind::MaintenanceEvent => "maintenance_event",
        MaritimeEvidenceKind::RecoveryEvent => "recovery_event",
    }
}

pub fn verify_maritime_successor(
    previous: &MaritimeEvidenceEnvelope,
    next: &MaritimeEvidenceEnvelope,
) -> Result<(), String> {
    previous.validate()?;
    next.validate()?;
    if previous.platform_id != next.platform_id {
        return Err("platform_id changed within maritime event chain".into());
    }
    if next.generation < previous.generation {
        return Err(format!(
            "generation regression: predecessor {}, successor {}",
            previous.generation, next.generation
        ));
    }
    let expected_sequence = previous
        .sequence
        .checked_add(1)
        .ok_or_else(|| "predecessor sequence overflow".to_string())?;
    if next.sequence != expected_sequence {
        return Err(format!(
            "sequence discontinuity: expected {}, got {}",
            expected_sequence, next.sequence
        ));
    }
    let expected = previous.content_digest()?;
    if next.previous_event_digest.as_deref() != Some(expected.as_str()) {
        return Err("previous_event_digest does not bind the predecessor".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(sequence: u64) -> MaritimeEvidenceEnvelope {
        MaritimeEvidenceEnvelope::new(
            "auv-01",
            7,
            sequence,
            1_000_000 + sequence,
            MaritimeEvidenceKind::HealthObservation,
            r#"{"severity":"healthy"}"#,
            "hal-evidence:abc",
        )
    }

    #[test]
    fn valid_payload_is_digestible() {
        let envelope = event(0);
        assert_eq!(envelope.validate(), Ok(()));
        assert_eq!(envelope.content_digest().unwrap().len(), 64);
    }

    #[test]
    fn malformed_payload_and_missing_evidence_fail_closed() {
        let mut malformed = event(0);
        malformed.payload_json = "not-json".into();
        assert!(malformed.validate().is_err());

        let mut padded_payload = event(0);
        padded_payload.payload_json = " {\"severity\":\"healthy\"}".into();
        assert!(padded_payload.validate().is_err());

        let mut missing = event(0);
        missing.evidence_binding.clear();
        assert!(missing.validate().is_err());
    }

    #[test]
    fn canonical_identifiers_and_reference_sets_are_required() {
        let mut padded_platform = event(0);
        padded_platform.platform_id = " auv-01".into();
        assert!(padded_platform.validate().is_err());

        let mut padded_evidence = event(0);
        padded_evidence.evidence_binding = "hal-evidence:abc ".into();
        assert!(padded_evidence.validate().is_err());

        let mut unsorted = event(0);
        unsorted.position_evidence_refs = vec!["position:z".into(), "position:a".into()];
        assert!(unsorted.validate().is_err());

        let mut duplicate = event(0);
        duplicate.position_evidence_refs = vec!["position:a".into(), "position:a".into()];
        assert!(duplicate.validate().is_err());

        let mut uppercase_predecessor = event(1);
        uppercase_predecessor.previous_event_digest = Some("A".repeat(64));
        assert!(uppercase_predecessor.validate().is_err());
    }

    #[test]
    fn sorted_unique_position_references_are_digestible() {
        let mut envelope = event(0);
        envelope.position_evidence_refs = vec![
            "mycelix-position:measurement:001".into(),
            "mycelix-position:measurement:002".into(),
        ];
        assert_eq!(envelope.validate(), Ok(()));
        assert_eq!(envelope.content_digest().unwrap().len(), 64);
    }

    #[test]
    fn serialized_envelope_must_fit_existing_bridge_payload_limit() {
        let mut envelope = event(0);
        // The inner payload remains under its coarse 60 KiB bound but the complete
        // serialized envelope cannot fit the generic bridge's 8 KiB payload field.
        envelope.payload_json = serde_json::to_string(&"x".repeat(8 * 1024)).unwrap();
        assert!(envelope.payload_json.len() < MAX_PAYLOAD_BYTES);
        let error = envelope.validate().unwrap_err();
        assert!(error.contains("Commons bridge payload limit"), "got: {error}");
        assert!(envelope
            .to_commons_event(AgentPubKey::from_raw_36(vec![0u8; 36]), Timestamp::from_micros(0))
            .is_err());
    }

    #[test]
    fn successor_binds_exact_predecessor_and_detects_tampering() {
        let first = event(41);
        let second = event(42).chain_after(&first).unwrap();
        assert_eq!(verify_maritime_successor(&first, &second), Ok(()));

        let mut tampered = first.clone();
        tampered.payload_json = r#"{"severity":"unsafe"}"#.into();
        assert!(verify_maritime_successor(&tampered, &second).is_err());
    }

    #[test]
    fn generation_can_stay_constant_or_advance_but_never_regress() {
        let first = event(9);

        let same = event(10).chain_after(&first).unwrap();
        assert_eq!(verify_maritime_successor(&first, &same), Ok(()));

        let mut advanced = event(10);
        advanced.generation = 8;
        let advanced = advanced.chain_after(&first).unwrap();
        assert_eq!(verify_maritime_successor(&first, &advanced), Ok(()));

        let mut regressed = event(10);
        regressed.generation = 6;
        assert!(regressed.chain_after(&first).is_err());

        let mut forged_regression = same;
        forged_regression.generation = 6;
        forged_regression.previous_event_digest = Some(first.content_digest().unwrap());
        assert!(verify_maritime_successor(&first, &forged_regression).is_err());
    }
}
