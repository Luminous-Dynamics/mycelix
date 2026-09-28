//! Runtime-neutral versioned semantic seam reference model.
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeliveryMode { Request, Event, Notification, Query, Acknowledgment }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReceiptStage {
    DispatchPrepared, DispatchAttempted, TransportAccepted, DeliveredOrObserved,
    RecipientAcknowledged, RecipientSemanticallyAdmitted, Rejected, Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SemanticDecision { Accepted, Rejected, Indeterminate, StaleSchema, PayloadMismatch, Unauthorized }

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SeamProfile {
    pub profile_id: String,
    pub profile_version: u32,
    pub sender_domain: String,
    pub receiver_domain: String,
    pub source_schema_version: String,
    pub delivery_mode: DeliveryMode,
    pub retry_idempotency_profile: String,
    pub ordering_guaranteed: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SeamEnvelope {
    pub profile_id: String,
    pub profile_version: u32,
    pub semantic_subject_id: String,
    pub payload_commitment: String,
    pub delivery_id: String,
    pub attempt_id: String,
    pub source_schema_version: String,
    pub origin: String,
    pub authority_reference: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Receipt {
    pub delivery_id: String,
    pub attempt_id: String,
    pub stage: ReceiptStage,
    pub semantic_decision: Option<SemanticDecision>,
}

pub fn validate_envelope(profile: &SeamProfile, envelope: &SeamEnvelope) -> SemanticDecision {
    if envelope.profile_id != profile.profile_id
        || envelope.profile_version != profile.profile_version
        || envelope.source_schema_version != profile.source_schema_version {
        return SemanticDecision::StaleSchema;
    }
    if envelope.semantic_subject_id.is_empty()
        || envelope.payload_commitment.is_empty()
        || envelope.delivery_id.is_empty()
        || envelope.attempt_id.is_empty() {
        return SemanticDecision::Rejected;
    }
    SemanticDecision::Accepted
}

pub fn admit_after_receipt(
    profile: &SeamProfile,
    envelope: &SeamEnvelope,
    receipt: &Receipt,
) -> SemanticDecision {
    let envelope_result = validate_envelope(profile, envelope);
    if envelope_result != SemanticDecision::Accepted {
        return envelope_result;
    }
    if receipt.delivery_id != envelope.delivery_id || receipt.attempt_id != envelope.attempt_id {
        return SemanticDecision::Rejected;
    }
    match receipt.stage {
        ReceiptStage::RecipientSemanticallyAdmitted => SemanticDecision::Accepted,
        ReceiptStage::Rejected => SemanticDecision::Rejected,
        ReceiptStage::Indeterminate
        | ReceiptStage::DispatchPrepared
        | ReceiptStage::DispatchAttempted
        | ReceiptStage::TransportAccepted
        | ReceiptStage::DeliveredOrObserved
        | ReceiptStage::RecipientAcknowledged => SemanticDecision::Indeterminate,
    }
}

/// A transport/provider receipt never grants executable authority.
pub fn receipt_grants_authority(_receipt: &Receipt) -> bool { false }

/// Retry may allocate a new attempt, but cannot mutate the logical delivery/payload.
pub fn retry_compatible(original: &SeamEnvelope, retry: &SeamEnvelope) -> bool {
    original.profile_id == retry.profile_id
        && original.profile_version == retry.profile_version
        && original.semantic_subject_id == retry.semantic_subject_id
        && original.payload_commitment == retry.payload_commitment
        && original.delivery_id == retry.delivery_id
}

/// Foreign recognition preserves the foreign origin instead of localizing it.
pub fn recognized_origin(origin: &str, recognizing_domain: &str) -> String {
    format!("foreign:{origin};recognized-by:{recognizing_domain}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> SeamProfile {
        SeamProfile {
            profile_id: "INTEGRAL-REF-IF-001".into(),
            profile_version: 1,
            sender_domain: "OAD".into(),
            receiver_domain: "COS".into(),
            source_schema_version: "oad.v1".into(),
            delivery_mode: DeliveryMode::Event,
            retry_idempotency_profile: "stable-delivery-id".into(),
            ordering_guaranteed: false,
        }
    }

    fn envelope() -> SeamEnvelope {
        SeamEnvelope {
            profile_id: "INTEGRAL-REF-IF-001".into(),
            profile_version: 1,
            semantic_subject_id: "design-1".into(),
            payload_commitment: "sha256:payload".into(),
            delivery_id: "delivery-1".into(),
            attempt_id: "attempt-1".into(),
            source_schema_version: "oad.v1".into(),
            origin: "node-a".into(),
            authority_reference: None,
        }
    }

    #[test]
    fn provider_acceptance_is_not_semantic_admission() {
        let e = envelope();
        let r = Receipt { delivery_id: "delivery-1".into(), attempt_id: "attempt-1".into(),
            stage: ReceiptStage::TransportAccepted, semantic_decision: None };
        assert_eq!(admit_after_receipt(&profile(), &e, &r), SemanticDecision::Indeterminate);
        assert!(!receipt_grants_authority(&r));
    }

    #[test]
    fn semantic_admission_requires_matching_identity() {
        let e = envelope();
        let r = Receipt { delivery_id: "other".into(), attempt_id: "attempt-1".into(),
            stage: ReceiptStage::RecipientSemanticallyAdmitted,
            semantic_decision: Some(SemanticDecision::Accepted) };
        assert_eq!(admit_after_receipt(&profile(), &e, &r), SemanticDecision::Rejected);
    }

    #[test]
    fn stale_profile_is_rejected() {
        let mut e = envelope();
        e.profile_version = 2;
        assert_eq!(validate_envelope(&profile(), &e), SemanticDecision::StaleSchema);
    }

    #[test]
    fn retry_preserves_payload_but_can_change_attempt() {
        let original = envelope();
        let mut retry = original.clone();
        retry.attempt_id = "attempt-2".into();
        assert!(retry_compatible(&original, &retry));
        retry.payload_commitment = "changed".into();
        assert!(!retry_compatible(&original, &retry));
    }

    #[test]
    fn foreign_recognition_does_not_localize_origin() {
        let value = recognized_origin("node-b", "node-a");
        assert!(value.starts_with("foreign:node-b"));
        assert!(value.contains("recognized-by:node-a"));
    }
}
