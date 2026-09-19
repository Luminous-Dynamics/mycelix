// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical qualification receipts and the receipt-authentication capability boundary.
//!
//! This crate intentionally contains no cryptographic verifier backend yet. It freezes
//! the canonical receipt identity, exact authentication policy, opaque capability, and
//! strict untrusted-wire parsing boundary that future separately-qualified backends may
//! use after full verification.

mod canonical;
mod capability;
mod policy;
mod predicate;
mod wire;

pub use canonical::{
    GitObjectIdParseErrorV1, GitObjectIdV1, MAX_RECEIPT_IDENTIFIER_BYTES_V1,
    MAX_RECEIPT_NONCLAIMS_V1, MAX_RECEIPT_NONCLAIM_BYTES_V1,
    QUALIFICATION_RECEIPT_CANONICALIZATION_PROFILE_V1,
    QualificationReceiptCanonicalizationV1, QualificationReceiptDigestV1,
    QualificationReceiptV1, QualificationResultV1, ReceiptCanonicalizationErrorV1,
    Sha256DigestParseErrorV1, Sha256DigestV1,
};
pub use capability::{
    AuthenticatedCapabilityConstructionErrorV1, AuthenticatedQualificationReceiptV1,
    AuthenticatedReceiptAuthorityV1, AuthenticationEvidenceSummaryV1,
    VerifiedAuthenticationContextV1, VerifiedFreshnessV1, VerifiedQualificationPredicateV1,
    VerifiedSignerIdentityV1, VerifiedTransparencyV1,
};
pub use policy::{
    AuthenticationFreshnessPolicyV1, AuthenticationPolicyErrorV1,
    ReceiptAuthenticationPolicyV1, SourceRevisionPolicyV1, TransparencyPolicyV1,
    VerifierProfileV1, WorkflowRevisionPolicyV1,
};
pub use predicate::{
    QUALIFICATION_ATTESTATION_PREDICATE_SCHEMA_V1,
    QualificationAttestationPredicateErrorV1, QualificationAttestationPredicateV1,
};
pub use wire::{
    MAX_UNTRUSTED_WIRE_JSON_BYTES_V1, MAX_UNTRUSTED_WIRE_STRING_BYTES_V1,
    MAX_UNTRUSTED_WORKFLOW_REVISIONS_V1, UntrustedWireParseErrorV1,
    parse_untrusted_authentication_policy_json_v1,
    parse_untrusted_qualification_predicate_json_v1,
};
