#![deny(deprecated)]

use mycelix_legacy_governance_v1::{
    AUTHORITY_SEMANTICS_VERSION, LegacyV1GovernanceEligibility, LegacyV1GovernanceRequirement,
};

#[test]
fn legacy_v1_public_types_are_non_deprecated_at_the_facade_boundary() {
    let _requirement: Option<LegacyV1GovernanceRequirement> = None;
    let _eligibility: Option<LegacyV1GovernanceEligibility> = None;
    assert_eq!(
        AUTHORITY_SEMANTICS_VERSION,
        "mycelix:governance:legacy-v1"
    );
}

#[cfg(feature = "hdk")]
#[test]
fn legacy_v1_gate_symbol_is_non_deprecated_for_downstream_callers() {
    let _gate: fn(
        &str,
        &LegacyV1GovernanceRequirement,
        &str,
    ) -> hdk::prelude::ExternResult<LegacyV1GovernanceEligibility> =
        mycelix_legacy_governance_v1::gate_legacy_v1;
}
