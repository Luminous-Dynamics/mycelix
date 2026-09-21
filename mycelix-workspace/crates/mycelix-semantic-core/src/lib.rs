#![forbid(unsafe_code)]

//! MYC-SEM-001A: primitive semantic identity references.
//!
//! This crate is deliberately dependency-light and domain-agnostic. It does not
//! establish truth, currentness, authority, decisions, execution, or semantic
//! equivalence. It only provides bounded identifiers and exact opaque references
//! that later qualified layers can bind.

mod model;

pub use model::{
    BoundedSemanticTextV1, Commitment32, MAX_SEMANTIC_TEXT_BYTES, SchemaRefV1,
    SemanticCoreError, SemanticIdV1, SemanticProfileRefV1,
};
