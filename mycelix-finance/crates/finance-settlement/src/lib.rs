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

pub use canonical::{
    CanonicalEncodingError, SETTLEMENT_COMMITMENT_PROFILE_REVISION,
    canonical_evaluation_context_bytes, canonical_evidence_bytes,
    canonical_finality_profile_bytes, canonical_observation_bytes,
    canonical_selected_evidence_frontier_bytes, evaluation_context_commitment,
    evidence_commitment, finality_profile_commitment, observation_commitment,
    selected_evidence_frontier_commitment,
};
pub use model::*;
pub use qualify::{
    SettlementInvalidationError, SettlementQualificationError, derive_invalidation,
    qualify_settlement,
};

#[cfg(test)]
mod tests;
