#![forbid(unsafe_code)]
//! Minimal normative kernel types for MYCELIX-ASSURE/V1.
//!
//! ASSURE-002B adds only L0 canonical CBOR admission. This crate still does not
//! validate assurance graphs, evaluate claims, establish effective authority,
//! or emit positive qualification results.

pub mod bounds;
pub mod codec;
pub mod digest;
pub mod ids;
pub mod time;

pub use bounds::KernelBounds;
pub use codec::{
    CanonicalCbor, DecodeFailure, DecodeFailureCode, DecodeFailureWitness, DecodeLimits,
    ForbiddenValueKind, ResourceLimit, decode_canonical,
};
pub use digest::Digest;
pub use ids::{IdError, NodeId};
pub use time::{TimeError, UnixMicros, ValidityEnd, ValidityInterval};
