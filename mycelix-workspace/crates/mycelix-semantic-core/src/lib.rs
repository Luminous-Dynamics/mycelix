#![forbid(unsafe_code)]

//! MYC-SEM-001: dependency-light semantic identity and environment primitives.
//!
//! This crate is deliberately domain-agnostic. It does not establish truth,
//! currentness, authority, decisions, execution, or semantic equivalence.
//! It provides exact references and explicit semantic-environment bindings that
//! later qualified layers can commit and consume.

mod model;

pub use model::{
    BoundedSemanticTextV1, Commitment32, MAX_SEMANTIC_TEXT_BYTES, SchemaRefV1,
    SemanticCoreError, SemanticEnvironmentV1, SemanticIdV1, SemanticProfileRefV1,
};
