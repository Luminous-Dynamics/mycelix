#![deny(unsafe_code)]

//! Rail-neutral settlement qualification contracts for Mycelix Finance.
//!
//! This crate deliberately does not own business authorization, commercial
//! satisfaction, legal discharge, or rail truth. It binds Finance-owned exact
//! amounts to explicit finality profiles and supplied evidence while preserving
//! unknown, conflicting, stale, reversal, and replay states.

mod model;
mod qualify;

pub use model::*;
pub use qualify::{qualify_settlement, SettlementQualificationError};

#[cfg(test)]
mod tests;
