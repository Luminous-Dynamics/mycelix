#![forbid(unsafe_code)]

//! FIN-SYNC-002A: observation-only settlement-graph evaluation.
//!
//! This crate classifies supplied **unqualified** per-leg observations against
//! one exact FIN-SYNC-001 graph. It deliberately cannot represent qualified
//! settlement completion, synchronization proof, dispatch authority, provider
//! truth, recovery authority, or compensation authority.

mod canonical;
mod evaluate;
mod model;

pub use evaluate::evaluate_observed_graph_v1;
pub use model::{
    DerivedLegObservationV1, EvalError, LegObservationInputV1, LegObservedDispositionV1,
    ObservationClassV1, ObservationEvaluationInputV1, ObservedGraphDispositionV1,
    ObservedGraphReceiptV1, SelectedObservationStreamV1, MAX_OBSERVATIONS_PER_LEG,
    MAX_OBSERVATION_STREAMS_PER_LEG, MAX_TOTAL_OBSERVATIONS,
};

#[cfg(test)]
mod tests;
