//! Explicit claim lifecycle semantics layered over constitutional consumption.
//!
//! Consumption owns single-finalization/effect safety. Temporal provenance owns
//! accepted evidence and closure history. Closure coverage decides whether a
//! revocation interval currently authorizes terminalization. This crate composes
//! those layers without deleting claim evidence.

mod invariants;
mod state;
mod transitions;
mod types;

pub use state::ClaimLifecycleState;
pub use types::*;
