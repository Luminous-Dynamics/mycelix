#![forbid(unsafe_code)]
//! Minimal normative kernel types for MYCELIX-ASSURE/V1.
//!
//! ASSURE-002A is intentionally deny-first. This crate does not decode
//! assurance capsules, evaluate claims, establish effective authority, or emit
//! positive qualification results.

pub mod bounds;
pub mod digest;
pub mod ids;
pub mod time;

pub use bounds::KernelBounds;
pub use digest::Digest;
pub use ids::{IdError, NodeId};
pub use time::{TimeError, UnixMicros, ValidityEnd, ValidityInterval};
