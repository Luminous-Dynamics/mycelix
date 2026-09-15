#![deny(unsafe_code)]

//! Typed, non-authorizing bindings between the Mycelix Business Fabric and Finance.
//!
//! Business owns decision, authorization, coordination, and attempt semantics.
//! Finance owns exact financial state, reservations, exposure, and settlement
//! qualification. This crate checks exact cross-domain bindings; it does not mint
//! authority, issue reservations, mutate balances, or declare business success.

mod binding;
mod model;
mod projection;

pub use binding::bind_finance_for_attempt;
pub use model::*;
pub use projection::*;

#[cfg(test)]
mod tests;
