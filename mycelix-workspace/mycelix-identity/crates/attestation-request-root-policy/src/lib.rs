// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical structural theorem for Mycelix attestation-request creation roots.
//!
//! This crate deliberately knows nothing about Holochain actions, DHT/network
//! completeness, entry serialization, credential issuance, or proof verification.
//! Adapters provide ordinary scalar/borrowed values and separately enforce action
//! author/index binding.

#![forbid(unsafe_code)]

pub use mycelix_attestation_request_policy::AttestationRequestStatusV1;

/// Current V1 textual request identifier bound used by legacy response APIs.
///
/// This is a compatibility bound, not the long-term authority design. Canonical
/// root-addressed operations should supersede free-form string authority.
pub const REQUEST_ID_MAX_LEN_V1: usize = 256;
pub const DID_MAX_LEN_V1: usize = 256;
pub const PURPOSE_MAX_LEN_V1: usize = 2048;

/// Holochain-free inputs needed to validate a canonical request root.
#[derive(Debug, Clone, Copy)]
pub struct AttestationRequestRootAssertionV1<'a> {
    pub id: &'a str,
    pub requester_did: &'a str,
    pub subject_did: &'a str,
    /// Number of requested K-Vector components. Component-set uniqueness is
    /// intentionally outside V1 until separately qualified.
    pub component_count: usize,
    pub purpose: &'a str,
    pub min_trust_score: Option<f32>,
    pub status: AttestationRequestStatusV1,
    pub created_at_micros: i64,
    pub expires_at_micros: i64,
}

/// Fail-closed structural defects in a canonical request root.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttestationRequestRootAssertionErrorV1 {
    RequestIdInvalid,
    RequesterDidInvalid,
    SubjectDidInvalid,
    SelfRequest,
    ComponentsEmpty,
    PurposeInvalid,
    RootStatusNotPending,
    MinTrustScoreNonFinite,
    MinTrustScoreOutOfBounds,
    InvalidValidityInterval,
}

fn valid_did_shape_v1(did: &str) -> bool {
    !did.is_empty() && did.len() <= DID_MAX_LEN_V1 && did.starts_with("did:")
}

/// Validate the structural assertion made by one canonical request creation root.
///
/// Security theorem:
///
/// - identifiers/DIDs/purpose are bounded under the V1 compatibility contract;
/// - requester and subject are distinct;
/// - at least one component is requested;
/// - creation root status is exactly Pending;
/// - optional numeric threshold is finite and within [0, 1];
/// - expiration is strictly after creation.
///
/// Actor binding and subject-index binding are intentionally adapter concerns.
pub fn validate_attestation_request_root_assertion_v1(
    root: AttestationRequestRootAssertionV1<'_>,
) -> Result<(), AttestationRequestRootAssertionErrorV1> {
    if root.id.is_empty() || root.id.len() > REQUEST_ID_MAX_LEN_V1 {
        return Err(AttestationRequestRootAssertionErrorV1::RequestIdInvalid);
    }
    if !valid_did_shape_v1(root.requester_did) {
        return Err(AttestationRequestRootAssertionErrorV1::RequesterDidInvalid);
    }
    if !valid_did_shape_v1(root.subject_did) {
        return Err(AttestationRequestRootAssertionErrorV1::SubjectDidInvalid);
    }
    if root.requester_did == root.subject_did {
        return Err(AttestationRequestRootAssertionErrorV1::SelfRequest);
    }
    if root.component_count == 0 {
        return Err(AttestationRequestRootAssertionErrorV1::ComponentsEmpty);
    }
    if root.purpose.is_empty() || root.purpose.len() > PURPOSE_MAX_LEN_V1 {
        return Err(AttestationRequestRootAssertionErrorV1::PurposeInvalid);
    }
    if root.status != AttestationRequestStatusV1::Pending {
        return Err(AttestationRequestRootAssertionErrorV1::RootStatusNotPending);
    }
    if let Some(score) = root.min_trust_score {
        if !score.is_finite() {
            return Err(AttestationRequestRootAssertionErrorV1::MinTrustScoreNonFinite);
        }
        if !(0.0..=1.0).contains(&score) {
            return Err(AttestationRequestRootAssertionErrorV1::MinTrustScoreOutOfBounds);
        }
    }
    if root.expires_at_micros <= root.created_at_micros {
        return Err(AttestationRequestRootAssertionErrorV1::InvalidValidityInterval);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const REQUESTER: &str = "did:mycelix:requester";
    const SUBJECT: &str = "did:mycelix:subject";

    fn valid_root<'a>() -> AttestationRequestRootAssertionV1<'a> {
        AttestationRequestRootAssertionV1 {
            id: "req-1",
            requester_did: REQUESTER,
            subject_did: SUBJECT,
            component_count: 1,
            purpose: "governance eligibility",
            min_trust_score: Some(0.5),
            status: AttestationRequestStatusV1::Pending,
            created_at_micros: 100,
            expires_at_micros: 1_000,
        }
    }

    #[test]
    fn valid_root_is_accepted() {
        assert_eq!(validate_attestation_request_root_assertion_v1(valid_root()), Ok(()));
    }

    #[test]
    fn identifier_boundary_is_frozen() {
        let id = "r".repeat(REQUEST_ID_MAX_LEN_V1);
        let mut root = valid_root();
        root.id = &id;
        assert_eq!(validate_attestation_request_root_assertion_v1(root), Ok(()));

        let id = "r".repeat(REQUEST_ID_MAX_LEN_V1 + 1);
        let mut root = valid_root();
        root.id = &id;
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::RequestIdInvalid)
        );
    }

    #[test]
    fn malformed_dids_and_self_request_fail_closed() {
        let mut root = valid_root();
        root.requester_did = "";
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::RequesterDidInvalid)
        );

        let subject = format!("did:{}", "s".repeat(DID_MAX_LEN_V1));
        let mut root = valid_root();
        root.subject_did = &subject;
        assert!(subject.len() > DID_MAX_LEN_V1);
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::SubjectDidInvalid)
        );

        let mut root = valid_root();
        root.subject_did = REQUESTER;
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::SelfRequest)
        );
    }

    #[test]
    fn components_and_purpose_are_bounded() {
        let mut root = valid_root();
        root.component_count = 0;
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::ComponentsEmpty)
        );

        let mut root = valid_root();
        root.purpose = "";
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::PurposeInvalid)
        );

        let purpose = "p".repeat(PURPOSE_MAX_LEN_V1 + 1);
        let mut root = valid_root();
        root.purpose = &purpose;
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::PurposeInvalid)
        );
    }

    #[test]
    fn every_non_pending_creation_state_is_rejected() {
        for status in [
            AttestationRequestStatusV1::Fulfilled,
            AttestationRequestStatusV1::Declined,
            AttestationRequestStatusV1::Expired,
            AttestationRequestStatusV1::Cancelled,
        ] {
            let mut root = valid_root();
            root.status = status;
            assert_eq!(
                validate_attestation_request_root_assertion_v1(root),
                Err(AttestationRequestRootAssertionErrorV1::RootStatusNotPending)
            );
        }
    }

    #[test]
    fn nonfinite_and_out_of_bounds_thresholds_fail_closed() {
        for score in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut root = valid_root();
            root.min_trust_score = Some(score);
            assert_eq!(
                validate_attestation_request_root_assertion_v1(root),
                Err(AttestationRequestRootAssertionErrorV1::MinTrustScoreNonFinite)
            );
        }

        for score in [-0.01, 1.01] {
            let mut root = valid_root();
            root.min_trust_score = Some(score);
            assert_eq!(
                validate_attestation_request_root_assertion_v1(root),
                Err(AttestationRequestRootAssertionErrorV1::MinTrustScoreOutOfBounds)
            );
        }

        for score in [0.0, 1.0] {
            let mut root = valid_root();
            root.min_trust_score = Some(score);
            assert_eq!(validate_attestation_request_root_assertion_v1(root), Ok(()));
        }
    }

    #[test]
    fn validity_interval_is_strictly_positive() {
        let mut root = valid_root();
        root.expires_at_micros = root.created_at_micros;
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::InvalidValidityInterval)
        );

        let mut root = valid_root();
        root.expires_at_micros = root.created_at_micros - 1;
        assert_eq!(
            validate_attestation_request_root_assertion_v1(root),
            Err(AttestationRequestRootAssertionErrorV1::InvalidValidityInterval)
        );
    }
}
