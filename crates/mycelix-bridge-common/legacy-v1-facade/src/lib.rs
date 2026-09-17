// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

#![forbid(unsafe_code)]
#![deny(deprecated)]

//! Explicit compatibility facade for Mycelix LegacyV1 governance semantics.
//!
//! This crate exists to separate **API retirement** from **semantic retirement**.
//! It does not implement CivicV2 and it must not translate, intersect, or
//! otherwise reinterpret LegacyV1 requirements. The HDK gate below is a
//! one-hop delegate to the frozen legacy `gate_consciousness` implementation.
//!
//! Callers may migrate to these names to remove deprecated-symbol pressure
//! while preserving the existing LegacyV1 authority contract. Migration from
//! LegacyV1 to CivicV2 is a separate, explicitly reviewed policy transition.

/// Stable identifier for the authority semantics exposed by this facade.
pub const AUTHORITY_SEMANTICS_VERSION: &str = "mycelix:governance:legacy-v1";

/// The exact legacy governance requirement type, exposed under an explicit
/// semantics-version name rather than the deprecated API name.
#[allow(deprecated)]
pub use mycelix_bridge_common::GovernanceRequirement as LegacyV1GovernanceRequirement;

/// Eligibility result returned by the existing LegacyV1 gate.
pub use mycelix_bridge_common::GovernanceEligibility as LegacyV1GovernanceEligibility;

/// Evaluate the exact existing LegacyV1 governance gate.
///
/// This function intentionally contains no conversion to `CivicRequirement`,
/// no call to `gate_civic`, and no additional admission condition. It delegates
/// directly to the existing legacy implementation so credential lookup, expiry,
/// auditing, refresh, rejection, and error behavior remain unchanged.
#[cfg(feature = "hdk")]
#[allow(deprecated)]
pub fn gate_legacy_v1(
    bridge_zome: &str,
    requirement: &LegacyV1GovernanceRequirement,
    action_name: &str,
) -> hdk::prelude::ExternResult<LegacyV1GovernanceEligibility> {
    mycelix_bridge_common::gate_consciousness(bridge_zome, requirement, action_name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn semantics_version_is_explicit_and_stable() {
        assert_eq!(AUTHORITY_SEMANTICS_VERSION, "mycelix:governance:legacy-v1");
    }
}
