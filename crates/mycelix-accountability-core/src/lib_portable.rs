// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable public boundary for reciprocal accountability.
//!
//! The original protocol implementation remains encapsulated below for its
//! semantic validation and notice projection. Cryptographic commitments are
//! deliberately re-exposed from `canonical`, whose encoding is independent of
//! Rust serializer implementation details. This transitional split keeps the
//! public API stable while the draft v0.1 protocol is hardened.

#[allow(dead_code)]
#[path = "lib.rs"]
mod legacy;

mod canonical;

pub use canonical::{
    canonical_policy_bytes, canonical_pre_attestation_receipt_bytes,
    policy_commitment, pre_attestation_receipt_commitment, purpose_commitment,
    AccountabilityCommitmentError, ACCOUNTABILITY_CANONICAL_CODEC,
};

pub use legacy::{
    evaluate_notification, project_subject_notice, validate_pre_attestation_receipt,
    validate_receipt, AccessReceipt, AccountabilityError, AccountabilityPolicy,
    AttestationRef, AttestationRole, AuthorityType, Commitment32, DelayApproval,
    DelayedNoticeSummary, DelayedNotificationAuthorization, DelayReason,
    DisclosureKind, DisclosureSummary, InferenceDisclosure, LegalAuthority,
    LookupOutcome, NotificationDirective, NotificationDisposition, PairwiseSubjectId,
    PurposeBinding, QueryBudgetCharge, RequesterNoticeLevel, RequestingPrincipal,
    SubjectInferenceNotice, SubjectNotice, SubjectNoticePolicy, SubjectNoticeRequester,
    SubjectRight, ACCOUNTABILITY_COMMITMENT_ALGORITHM,
    RECIPROCAL_ACCOUNTABILITY_PROTOCOL_VERSION,
};
