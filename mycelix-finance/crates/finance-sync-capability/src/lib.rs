#![forbid(unsafe_code)]

//! FIN-SYNC-003A: static rail-capability semantics.
//!
//! This crate describes what one exact rail/adapter/profile combination is
//! designed to support. It deliberately does not establish current provider
//! capability, credential validity, execution authority, dispatch, settlement,
//! or legal/commercial finality.

mod build;
mod canonical;
mod model;

pub use build::build_rail_capability_profile_v1;
pub use model::{
    AtomicityCapabilityV1, CancelCapabilityV1, CapabilityError, CapacityLockCapabilityV1,
    CommitCapabilityV1, EvidenceCapabilityV1, IdempotencyCollisionBehaviorV1,
    IdempotencyKeyScopeV1, IdempotencyMechanismV1, IdempotencyProfileV1,
    IdempotencySemanticScopeV1, PrepareCapabilityV1, QueryCapabilityV1,
    RailCapabilityProfileInputV1, RailCapabilityProfileV1, ResourceProfileV1,
    RetentionHorizonV1, ReversalCapabilityV1, TimingProfileV1, UnknownOutcomeRetryV1,
    MAX_CAPABILITIES_PER_DIMENSION, MAX_PROFILE_REFS,
};

#[cfg(test)]
mod tests;
