#![deny(unsafe_code)]

//! Canonical reservation commitments and deterministic lifecycle semantics for
//! Mycelix Finance.
//!
//! This crate defines what a Finance reservation descriptor and lifecycle state
//! mean. It does not authenticate Finance issuance, persist concurrent state,
//! mutate balances, establish settlement finality, or grant Business authority.

mod descriptor;
mod lifecycle;

pub use descriptor::*;
pub use lifecycle::*;

#[cfg(test)]
mod tests;
