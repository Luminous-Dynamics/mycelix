//! Candidate/reference SPEC-IF-01 OAD -> COS interface semantics.
//!
//! This module is deliberately not a ratified Integral API. It turns the currently
//! public Phase-0 requirements (versioning, authentication, errors, retry behavior,
//! and idempotency) into a small executable reference model. Authentication proves
//! who/which principal is speaking; authorization is a separate explicit fact.
//! Transport delivery never becomes semantic admission or production authority.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceStatus {
    PublicDraftReference,
    CandidateInterface,
    RatifiedSchema,
    Implementation,
    ConformanceEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Authn {
    Missing,
    Invalid,
    Valid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Authz {
    Missing,
    Denied,
    Granted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeliveryOutcome {
    TransportAccepted,
    Delivered,
    SemanticAdmitted,
    Rejected,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterfaceDecision {
    Admitted,
    DuplicateIdempotent,
    RejectedStaleSchema,
    RejectedPayloadMutation,
    RejectedIdentityMismatch,
    RejectedAuthentication,
    RejectedAuthorization,
    RejectedAuthorizationReference,
    RejectedCertification,
    RejectedSemanticDelivery,
    RejectedStaleDesign,
    RejectedSupersededDesign,
    Indeterminate,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InterfaceProfile {
    pub profile_id: &'static str,
    pub major: u16,
    pub minor: u16,
    pub source_status: SourceStatus,
    pub producer_schema: &'static str,
    pub consumer_schema: &'static str,
}

impl InterfaceProfile {
    pub const CANDIDATE_V1: Self = Self {
        profile_id: "INTEGRAL-SPEC-IF-01-OAD-COS-CANDIDATE",
        major: 1,
        minor: 0,
        source_status: SourceStatus::CandidateInterface,
        producer_schema: "CertifiedDesign-public-draft",
        consumer_schema: "COS-production-basis-reference",
    };
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DesignEnvelope {
    pub design_id: &'static str,
    pub design_generation: u64,
    pub schema_version: &'static str,
    pub certified: bool,
    pub superseded: bool,
    pub logical_delivery_id: &'static str,
    pub attempt_id: &'static str,
    pub payload_commitment: &'static str,
    pub origin_node: &'static str,
    pub authorization_ref: Option<&'static str>,
    pub presented_authorization_ref: Option<&'static str>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Receipt {
    pub logical_delivery_id: &'static str,
    pub attempt_id: &'static str,
    pub outcome: DeliveryOutcome,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdmissionState {
    pub admitted_delivery: Option<&'static str>,
    pub admitted_payload: Option<&'static str>,
}

pub fn authenticate(authn: Authn) -> InterfaceDecision {
    match authn {
        Authn::Valid => InterfaceDecision::Admitted,
        Authn::Missing | Authn::Invalid => InterfaceDecision::RejectedAuthentication,
    }
}

pub fn authorize(authz: Authz) -> InterfaceDecision {
    match authz {
        Authz::Granted => InterfaceDecision::Admitted,
        Authz::Missing | Authz::Denied => InterfaceDecision::RejectedAuthorization,
    }
}

pub fn validate_envelope(
    profile: &InterfaceProfile,
    envelope: &DesignEnvelope,
    authn: Authn,
    authz: Authz,
    expected_generation: u64,
    expected_schema: &str,
    expected_authorization_ref: Option<&str>,
) -> InterfaceDecision {
    if authenticate(authn) != InterfaceDecision::Admitted {
        return InterfaceDecision::RejectedAuthentication;
    }
    if authorize(authz) != InterfaceDecision::Admitted {
        return InterfaceDecision::RejectedAuthorization;
    }
    if authz == Authz::Granted {
        match (envelope.authorization_ref, envelope.presented_authorization_ref, expected_authorization_ref) {
            (Some(expected), Some(presented), Some(required)) if expected == presented && presented == required => {}
            _ => return InterfaceDecision::RejectedAuthorizationReference,
        }
    }
    if envelope.schema_version != expected_schema {
        return InterfaceDecision::RejectedStaleSchema;
    }
    if envelope.design_generation != expected_generation {
        return InterfaceDecision::RejectedStaleDesign;
    }
    if envelope.superseded {
        return InterfaceDecision::RejectedSupersededDesign;
    }
    if !envelope.certified {
        return InterfaceDecision::RejectedCertification;
    }
    if profile.source_status != SourceStatus::CandidateInterface
        && profile.source_status != SourceStatus::RatifiedSchema
        && profile.source_status != SourceStatus::Implementation
        && profile.source_status != SourceStatus::ConformanceEvidence
    {
        return InterfaceDecision::RejectedStaleSchema;
    }
    InterfaceDecision::Admitted
}

pub fn admit_after_receipt(
    state: &AdmissionState,
    receipt: &Receipt,
    envelope: &DesignEnvelope,
    validation: InterfaceDecision,
) -> InterfaceDecision {
    if !matches!(validation, InterfaceDecision::Admitted) {
        return validation;
    }

    match receipt.outcome {
        DeliveryOutcome::TransportAccepted | DeliveryOutcome::Delivered => {
            InterfaceDecision::Indeterminate
        }
        DeliveryOutcome::Rejected => InterfaceDecision::RejectedSemanticDelivery,
        DeliveryOutcome::Indeterminate => InterfaceDecision::Indeterminate,
        DeliveryOutcome::SemanticAdmitted => {
            if receipt.logical_delivery_id != envelope.logical_delivery_id {
                return InterfaceDecision::RejectedIdentityMismatch;
            }
            match (state.admitted_delivery, state.admitted_payload) {
                (Some(id), Some(payload))
                    if id == envelope.logical_delivery_id
                        && payload == envelope.payload_commitment =>
                {
                    InterfaceDecision::DuplicateIdempotent
                }
                (Some(id), Some(_)) if id == envelope.logical_delivery_id => {
                    InterfaceDecision::RejectedPayloadMutation
                }
                _ => InterfaceDecision::Admitted,
            }
        }
    }
}

/// Returns whether a retry is safe at the semantic-identity layer.
///
/// A retry may change attempt_id, but MUST retain logical_delivery_id and the
/// exact payload commitment. This function does not authorize execution.
pub fn retry_compatible(first: &DesignEnvelope, retry: &DesignEnvelope) -> bool {
    first.logical_delivery_id == retry.logical_delivery_id
        && first.payload_commitment == retry.payload_commitment
        && first.design_id == retry.design_id
        && first.design_generation == retry.design_generation
        && first.schema_version == retry.schema_version
}

/// Transport or semantic receipts never mint production authority.
pub const fn receipt_grants_production_authority(_receipt: &Receipt) -> bool {
    false
}

/// A source origin is immutable through recognition/admission.
pub fn recognized_origin(envelope: &DesignEnvelope) -> &'static str {
    envelope.origin_node
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(super) fn envelope() -> DesignEnvelope {
        DesignEnvelope {
            design_id: "design-1",
            design_generation: 7,
            schema_version: "oad-certified-design/0.1-draft",
            certified: true,
            superseded: false,
            logical_delivery_id: "delivery-1",
            attempt_id: "attempt-1",
            payload_commitment: "sha256:abc",
            origin_node: "node-a",
            authorization_ref: Some("authz-1"),
            presented_authorization_ref: Some("authz-1"),
        }
    }

    #[test]
    fn valid_authentication_and_authorization_are_separate() {
        assert_eq!(authenticate(Authn::Valid), InterfaceDecision::Admitted);
        assert_eq!(authorize(Authz::Missing), InterfaceDecision::RejectedAuthorization);
    }

    #[test]
    fn stale_schema_fails_closed() {
        let mut e = envelope();
        e.schema_version = "oad-certified-design/0.0";
        assert_eq!(
            validate_envelope(&InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7, "oad-certified-design/0.1-draft", Some("authz-1")),
            InterfaceDecision::RejectedStaleSchema
        );
    }

    #[test]
    fn transport_acceptance_is_not_semantic_admission() {
        let e = envelope();
        let r = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-1", outcome: DeliveryOutcome::TransportAccepted };
        let state = AdmissionState { admitted_delivery: None, admitted_payload: None };
        assert_eq!(admit_after_receipt(&state, &r, &e, InterfaceDecision::Admitted), InterfaceDecision::Indeterminate);
    }

    #[test]
    fn semantic_admission_requires_matching_logical_identity() {
        let e = envelope();
        let r = Receipt { logical_delivery_id: "delivery-other", attempt_id: "attempt-1", outcome: DeliveryOutcome::SemanticAdmitted };
        let state = AdmissionState { admitted_delivery: None, admitted_payload: None };
        assert_eq!(admit_after_receipt(&state, &r, &e, InterfaceDecision::Admitted), InterfaceDecision::RejectedIdentityMismatch);
    }

    #[test]
    fn retry_may_change_attempt_but_not_logical_delivery_or_payload() {
        let first = envelope();
        let mut retry = first.clone();
        retry.attempt_id = "attempt-2";
        assert!(retry_compatible(&first, &retry));
        retry.payload_commitment = "sha256:mutated";
        assert!(!retry_compatible(&first, &retry));
    }

    #[test]
    fn duplicate_semantic_admission_is_idempotent() {
        let e = envelope();
        let r = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-2", outcome: DeliveryOutcome::SemanticAdmitted };
        let state = AdmissionState { admitted_delivery: Some("delivery-1"), admitted_payload: Some("sha256:abc") };
        assert_eq!(admit_after_receipt(&state, &r, &e, InterfaceDecision::Admitted), InterfaceDecision::DuplicateIdempotent);
    }

    #[test]
    fn same_logical_delivery_with_mutated_payload_is_rejected() {
        let mut e = envelope();
        e.payload_commitment = "sha256:mutated";
        let r = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-2", outcome: DeliveryOutcome::SemanticAdmitted };
        let state = AdmissionState { admitted_delivery: Some("delivery-1"), admitted_payload: Some("sha256:abc") };
        assert_eq!(admit_after_receipt(&state, &r, &e, InterfaceDecision::Admitted), InterfaceDecision::RejectedPayloadMutation);
    }

    #[test]
    fn unknown_receipt_never_becomes_definite_failure() {
        let e = envelope();
        let r = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-1", outcome: DeliveryOutcome::Indeterminate };
        let state = AdmissionState { admitted_delivery: None, admitted_payload: None };
        assert_eq!(admit_after_receipt(&state, &r, &e, InterfaceDecision::Admitted), InterfaceDecision::Indeterminate);
    }

    #[test]
    fn foreign_origin_is_preserved() {
        let e = envelope();
        assert_eq!(recognized_origin(&e), "node-a");
    }

    #[test]
    fn receipts_never_grant_production_authority() {
        let r = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-1", outcome: DeliveryOutcome::SemanticAdmitted };
        assert!(!receipt_grants_production_authority(&r));
    }

    #[test]
    fn stale_or_superseded_designs_are_rejected_before_admission() {
        let mut e = envelope();
        e.superseded = true;
        assert_eq!(
            validate_envelope(&InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7, "oad-certified-design/0.1-draft"),
            InterfaceDecision::RejectedSupersededDesign
        );
    }
}

#[cfg(test)]
mod authorization_reference_tests {
    use super::*;

    #[test]
    fn granted_authorization_requires_explicit_matching_reference() {
        let mut e = super::tests::envelope();
        e.presented_authorization_ref = None;
        assert_eq!(validate_envelope(&InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7, "oad-certified-design/0.1-draft", Some("authz-1")), InterfaceDecision::RejectedAuthorizationReference);
    }

    #[test]
    fn certification_failure_is_not_mislabeled_as_authorization_failure() {
        let mut e = super::tests::envelope();
        e.certified = false;
        assert_eq!(validate_envelope(&InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7, "oad-certified-design/0.1-draft", Some("authz-1")), InterfaceDecision::RejectedCertification);
    }

    #[test]
    fn known_recipient_rejection_is_not_indeterminate() {
        let e = super::tests::envelope();
        let r = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-1", outcome: DeliveryOutcome::Rejected };
        let state = AdmissionState { admitted_delivery: None, admitted_payload: None };
        assert_eq!(admit_after_receipt(&state, &r, &e, InterfaceDecision::Admitted), InterfaceDecision::RejectedSemanticDelivery);
    }

    #[test]
    fn wrong_authorization_reference_is_rejected() {
        let mut e = super::tests::envelope();
        e.presented_authorization_ref = Some("authz-other");
        assert_eq!(validate_envelope(&InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7, "oad-certified-design/0.1-draft", Some("authz-1")), InterfaceDecision::RejectedAuthorizationReference);
    }
}
