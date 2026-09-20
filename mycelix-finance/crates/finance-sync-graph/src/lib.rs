#![forbid(unsafe_code)]

//! FIN-SYNC-001: canonical multi-leg settlement graph semantics.
//!
//! This crate is deliberately pure and dependency-light. It does not dispatch
//! external effects, authenticate authority, reserve capacity, qualify per-leg
//! settlement, or claim PvP/DvP atomicity. It only constructs and commits to a
//! bounded canonical graph over one already-identified economic effect.

mod build;
mod canonical;
mod model;

pub use build::build_settlement_graph_v1;
pub use model::{
    BoundedText, Commitment32, CoordinationGroupClassV1, CoordinationGroupSpecV1,
    CoordinationGroupV1, DependencySpecV1, GraphError, SemanticProfileRefV1,
    SettlementDependencyV1, SettlementGraphInputV1, SettlementGraphV1,
    SettlementLegSpecV1, SettlementLegV1, MAX_DEPENDENCIES, MAX_GROUPS,
    MAX_GROUP_MEMBERS, MAX_LEGS, MAX_TEXT_BYTES, MIN_LEGS,
};

#[cfg(test)]
mod tests;
