#![deny(unsafe_code)]

//! Rail-neutral settlement qualification contracts for Mycelix Finance.
//!
//! This crate deliberately does not own business authorization, commercial
//! satisfaction, legal discharge, provider truth, or runtime clock authority.
//! It binds exact Finance amounts to explicit finality policies, exact evidence,
//! and an explicit deterministic evaluation context.

mod canonical;
mod model;
mod qualify;
mod receipt;

pub use canonical::{
    CanonicalEncodingError, SETTLEMENT_COMMITMENT_PROFILE_REVISION,
    canonical_evaluation_context_bytes, canonical_evidence_bytes, canonical_finality_profile_bytes,
    canonical_observation_bytes, canonical_selected_evidence_frontier_bytes,
    evaluation_context_commitment, evidence_commitment, finality_profile_commitment,
    observation_commitment, selected_evidence_frontier_commitment,
};
pub use model::*;
pub use qualify::{
    SettlementInvalidationError, SettlementQualificationError, derive_invalidation,
    qualify_settlement,
};
pub use receipt::{
    QualifiedSettlementReceipt, SettlementInvalidationReceipt, SettlementInvalidationReceiptError,
    SettlementReceiptError, canonical_settlement_invalidation_receipt_bytes,
    derive_invalidation_with_receipt, qualify_settlement_with_receipt,
    settlement_invalidation_receipt_commitment,
};

#[cfg(test)]
mod metamorphic_tests;
#[cfg(test)]
mod receipt_tests;
#[cfg(test)]
mod receipt_vector_tests;
#[cfg(test)]
mod reference_ordering_vector_tests;
#[cfg(test)]
mod tests;
