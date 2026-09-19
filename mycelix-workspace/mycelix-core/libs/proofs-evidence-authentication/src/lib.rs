// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical qualification receipts and the receipt-authentication capability boundary.
//!
//! This crate intentionally contains no cryptographic verifier backend yet. It freezes
//! the canonical receipt identity, exact authentication policy, and opaque capability
//! shape that future separately-qualified backends may construct after full verification.

mod canonical;
mod capability;
mod policy;

pub use canonical::{
    GitObjectIdParseErrorV1, GitObjectIdV1, QualificationReceiptCanonicalizationV1,
    QualificationReceiptDigestV1, QualificationReceiptV1, QualificationResultV1,
    QUALIFICATION_RECEIPT_CANONICALIZATION_PROFILE_V1, ReceiptCanonicalizationErrorV1,
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
