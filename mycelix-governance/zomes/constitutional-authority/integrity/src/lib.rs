// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! DNA-bound constitutional authority root.

use hdi::prelude::*;
use mycelix_constitution_genesis::ConstitutionalGenesisManifest;

#[dna_properties]
#[derive(Debug, Clone, PartialEq)]
pub struct GovernanceDnaProperties {
    pub constitutional_genesis: ConstitutionalGenesisManifest,
}

impl GovernanceDnaProperties {
    pub fn validate(&self) -> Result<(), String> {
        self.constitutional_genesis
            .validate()
            .map_err(|error| format!("Invalid constitutional genesis manifest: {error}"))
    }
}

/// Fail the cell's genesis if its DNA properties do not carry a complete,
/// internally valid constitutional root. This does not inspect mutable DHT state.
#[hdk_extern]
pub fn genesis_self_check(_: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    let properties = match GovernanceDnaProperties::try_from_dna_properties() {
        Ok(properties) => properties,
        Err(error) => {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Constitutional DNA properties are missing or undecodable: {error}"
            )))
        }
    };

    match properties.validate() {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(error)),
    }
}

/// This zome defines no mutable application entries. Its authority comes from
/// DNA properties, which are part of the DNA modifiers/hash rather than DHT data.
#[hdk_extern]
pub fn validate(_: Op) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_constitution_genesis::{Digest32, PROTOCOL_VERSION};

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn rejects_malformed_manifest_before_runtime_use() {
        let properties = GovernanceDnaProperties {
            constitutional_genesis: ConstitutionalGenesisManifest {
                protocol_version: PROTOCOL_VERSION.into(),
                institution_id: "institution:test".into(),
                constitutional_rulebook_id: "rulebook:constitution".into(),
                constitutional_rulebook_version: "1".into(),
                constitutional_rulebook_digest: digest(1),
                constitutional_rulebook_digest_profile: "mycelix-rulebook-v1".into(),
                genesis_charter_digest: Digest32([0; 32]),
                genesis_charter_digest_profile: "mycelix-charter-v1".into(),
                genesis_parameter_manifest_digest: digest(3),
                genesis_parameter_manifest_profile: "mycelix-constitution-params-v1".into(),
                amendment_authority_policy_id: "policy:amendment:v1".into(),
                amendment_authority_policy_digest: digest(4),
                amendment_authority_policy_profile: "mycelix-authority-policy-v1".into(),
                bootstrap_authorities: vec![],
                genesis_epoch: 1,
            },
        };
        assert!(properties.validate().is_err());
    }
}
