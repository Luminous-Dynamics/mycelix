#![forbid(unsafe_code)]

//! MYC-SEM-001: dependency-light semantic identity, environment, and
//! language-neutral commitment primitives.
//!
//! This crate is deliberately domain-agnostic. It does not establish truth,
//! currentness, authority, decisions, execution, migration equivalence, or
//! translation authority.

mod canonical;
mod model;

pub use canonical::{
    SEMANTIC_COMMITMENT_PROFILE_ID, SEMANTIC_COMMITMENT_PROFILE_REVISION,
    SemanticCommitmentProfileV1,
};
pub use model::{
    BoundedSemanticTextV1, Commitment32, MAX_SEMANTIC_TEXT_BYTES, SchemaRefV1,
    SemanticCoreError, SemanticEnvironmentRefV1, SemanticEnvironmentV1, SemanticIdV1,
    SemanticProfileRefV1, SemanticSubjectRefV1,
};
