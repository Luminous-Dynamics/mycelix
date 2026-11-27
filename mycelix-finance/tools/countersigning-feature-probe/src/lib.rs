#![deny(unsafe_code)]

//! Compile-only probe for the optional Holochain countersigning API.
//!
//! This crate is deliberately not included in the Finance DNA. It verifies that
//! the pinned HDK/HDI versions expose the feature-gated preflight host call when
//! the explicit unstable Cargo features are requested. It does not run a
//! countersigning session or make a safety/finality claim.

use hdk::prelude::*;

/// Minimal bridge to the feature-gated Holochain preflight host function.
///
/// The integration harness must distribute the same request to every required
/// signer, collect every response, and commit the exact shared action set. This
/// probe only ensures the API is available at compile time.
#[hdk_extern]
pub fn accept_preflight(
    request: PreflightRequest,
) -> ExternResult<PreflightRequestAcceptance> {
    accept_countersigning_preflight_request(request)
}
