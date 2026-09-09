// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Typed payload contract for mission-neutral maritime evidence carried through
//! the existing Mycelix bridge event infrastructure.
//!
//! This module deliberately does **not** create a new Holochain entry type or a
//! new maritime governance silo. `MaritimeEvidenceEnvelope` is serialized into
//! the payload of the existing schema-versioned `BridgeEventEntry` using the
//! existing `transport` bridge domain and a dedicated event type.
//!
//! It also does not replace `mycelix-position`: position/navigation measurements
//! remain owned by that body-agnostic library and are referenced here by opaque
//! evidence IDs when useful.

use serde::{Deserialize, Serialize};

/// Existing bridge domain used for maritime evidence events.
///
/// This does not imply a call into the dormant transport zomes; it is only the
/// domain label on the generic bridge event envelope.
pub const MARITIME_BRIDGE_DOMAIN: &str = "transport";

/// Dedicated event type carried inside the existing bridge event schema.
pub const MARITIME_EVENT_TYPE_V1: &str = "maritime_evidence_v1";

pub const MARITIME_SCHEMA_V1: u8 = 1;
pub const MAX_PLATFORM_ID_BYTES: usize = 256;
pub const MAX_EVIDENCE_BINDING_BYTES: usize = 1024;
pub const MAX_PAYLOAD_BYTES: usize = 60 * 1024;
pub const MAX_REFERENCE_COUNT: usize = 64;
pub const MAX_REFERENCE_BYTES: usize = 512;

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

/// Portable event payload suitable for intermittent/store-and-forward transport.
///
/// `payload_json` is application-owned JSON. The bridge verifies boundedness and
/// chain continuity, not domain-specific payload meaning. `evidence_binding` is
/// an opaque reference to the evidence/signature/attestation system that produced
/// the claim; this crate does not pretend a BLAKE3 hash is an authentication proof.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MaritimeEvidenceEnvelope {
    pub schema_version: u8,
    pub platform_id: String,
    /// Software/configuration generation reported by the platform.
    pub generation: u64,
    /// Monotonically increasing sequence within this platform event stream.
    pub sequence: u64,
    pub observed_at_us: u64,
    pub kind: MaritimeEvidenceKind,
    pub payload_json: String,
    /// Opaque binding to upstream evidence (Xenia session transcript, HAL evidence,
    /// signed authority record, etc.). Verification remains with the owning system.
    pub evidence_binding: String,
    /// Opaque references to existing Mycelix Position measurements/estimates.
    #[serde(default)]
    pub position_evidence_refs: Vec<String>,
    /// Digest of the previous maritime envelope in this platform's store-forward
    /// chain. `None` marks a chain root.
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
        if self.platform_id.trim().is_empty() {
            return Err("platform_id cannot be empty".into());
        }
        if self.platform_id.len() > MAX_PLATFORM_ID_BYTES {
            return Err(format!(
                "platform_id exceeds {} bytes",
                MAX_PLATFORM_ID_BYTES
            ));
        }
        if self.payload_json.len() > MAX_PAYLOAD_BYTES {
            return Err(format!("payload_json exceeds {} bytes", MAX_PAYLOAD_BYTES));
        }
        if serde_json::from_str::<serde_json::Value>(&self.payload_json).is_err() {
            return Err("payload_json must contain valid JSON".into());
        }
        if self.evidence_binding.trim().is_empty() {
            return Err("evidence_binding cannot be empty".into());
        }
        if self.evidence_binding.len() > MAX_EVIDENCE_BINDING_BYTES {
            return Err(format!(
                "evidence_binding exceeds {} bytes",
                MAX_EVIDENCE_BINDING_BYTES
            ));
        }
        if self.position_evidence_refs.len() > MAX_REFERENCE_COUNT {
            return Err(format!(
                "too many position evidence references (max {})",
                MAX_REFERENCE_COUNT
            ));
        }
        for reference in &self.position_evidence_refs {
            if reference.trim().is_empty() {
                return Err("position evidence references cannot be empty".into());
            }
            if reference.len() > MAX_REFERENCE_BYTES {
                return Err(format!(
                    "position evidence reference exceeds {} bytes",
                    MAX_REFERENCE_BYTES
                ));
            }
        }
        if self
            .previous_event_digest
            .as_ref()
            .is_some_and(|digest| digest.trim().is_empty() || digest.len() > MAX_REFERENCE_BYTES)
        {
            return Err("previous_event_digest is empty or oversized".into());
        }
        Ok(())
    }

    /// Domain-separated content digest for store-forward continuity and dedupe.
    ///
    /// This is an integrity identifier, not an identity/authentication proof.
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
            None => hasher.update(&[0]),
        }
        Ok(hasher.finalize().to_hex().to_string())
    }

    /// Prepare this event as the direct successor of `previous`.
    pub fn chain_after(mut self, previous: &Self) -> Result<Self, String> {
        if self.platform_id != previous.platform_id {
            return Err("cannot chain maritime events from different platforms".into());
        }
        if self.sequence != previous.sequence.saturating_add(1) {
            return Err(format!(
                "successor sequence must be {}, got {}",
                previous.sequence.saturating_add(1),
                self.sequence
            ));
        }
        self.previous_event_digest = Some(previous.content_digest()?);
        self.validate()?;
        Ok(self)
    }
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

/// Verify that `next` is the direct store-forward successor of `previous`.
///
/// Generation is intentionally allowed to change across the link: software update
/// transitions should remain part of the same evidence history instead of silently
/// starting an unrelated chain.
pub fn verify_maritime_successor(
    previous: &MaritimeEvidenceEnvelope,
    next: &MaritimeEvidenceEnvelope,
) -> Result<(), String> {
    previous.validate()?;
    next.validate()?;
    if previous.platform_id != next.platform_id {
        return Err("platform_id changed within maritime event chain".into());
    }
    if next.sequence != previous.sequence.saturating_add(1) {
        return Err(format!(
            "sequence discontinuity: expected {}, got {}",
            previous.sequence.saturating_add(1),
            next.sequence
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
    fn valid_payload_is_bounded_and_digestible() {
        let envelope = event(0);
        assert_eq!(envelope.validate(), Ok(()));
        let digest = envelope.content_digest().unwrap();
        assert_eq!(digest.len(), 64);
    }

    #[test]
    fn malformed_json_and_missing_evidence_fail_closed() {
        let mut envelope = event(0);
        envelope.payload_json = "not-json".into();
        assert!(envelope.validate().is_err());

        let mut envelope = event(0);
        envelope.evidence_binding.clear();
        assert!(envelope.validate().is_err());
    }

    #[test]
    fn store_forward_successor_binds_exact_predecessor() {
        let first = event(41);
        let second = event(42).chain_after(&first).unwrap();
        assert_eq!(verify_maritime_successor(&first, &second), Ok(()));

        let mut tampered = first.clone();
        tampered.payload_json = r#"{"severity":"unsafe"}"#.into();
        assert!(verify_maritime_successor(&tampered, &second).is_err());
    }

    #[test]
    fn sequence_gap_is_rejected() {
        let first = event(1);
        assert!(event(3).chain_after(&first).is_err());
    }

    #[test]
    fn generation_transition_can_remain_in_same_evidence_chain() {
        let first = event(9);
        let mut next = event(10);
        next.generation = 8;
        let next = next.chain_after(&first).unwrap();
        assert_eq!(verify_maritime_successor(&first, &next), Ok(()));
    }
}
