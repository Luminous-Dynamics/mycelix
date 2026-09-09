// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Hash-bound integrity admission for the Identity V2 historical time-policy authority constitution.
//!
//! This integrity zome is the authority-critical enforcement home for the time-policy authority
//! protocol. Its first theorem is intentionally narrow: the executing DNA must carry one exact
//! #428 bootstrap-property envelope whose semantic declaration belongs to the expected Identity V2
//! constitution family/version before genesis is admitted.
//!
//! Success means only that the committed DNA properties are a well-formed canonical bootstrap
//! declaration for this protocol constitution. It does not establish deployment acceptance,
//! bootstrap legitimacy, signer authority/currentness, transition authenticity, policy currentness,
//! or trusted time.

#![forbid(unsafe_code)]

use hdi::prelude::*;
use mycelix_historical_activation_time_policy_authority_bootstrap_property_wire_policy::{
    decode_historical_activation_time_policy_authority_bootstrap_property_v2,
    HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2,
};

pub const IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_ID: &str =
    "mycelix-identity-v2-policy-authority";
pub const IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_VERSION: u32 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityV2PolicyAuthorityGenesisError {
    PropertyWire(HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2),
    ConstitutionIdMismatch,
    ConstitutionVersionMismatch,
}

/// Validate only the hash-bound constitutional property semantics required for this integrity zome.
///
/// Root key material and scope remain provisioned inputs committed by the DNA properties. This
/// function does not promote them into legitimate/current authority and does not derive the running
/// DNA authority-domain identity.
pub fn validate_identity_v2_policy_authority_genesis_property(
    property_bytes: &[u8],
) -> Result<(), IdentityV2PolicyAuthorityGenesisError> {
    let declaration = decode_historical_activation_time_policy_authority_bootstrap_property_v2(
        property_bytes,
    )
    .map_err(IdentityV2PolicyAuthorityGenesisError::PropertyWire)?;

    if declaration.constitution_id() != IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_ID {
        return Err(IdentityV2PolicyAuthorityGenesisError::ConstitutionIdMismatch);
    }
    if declaration.constitution_version() != IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_VERSION {
        return Err(IdentityV2PolicyAuthorityGenesisError::ConstitutionVersionMismatch);
    }

    Ok(())
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    let info = dna_info()?;
    match validate_identity_v2_policy_authority_genesis_property(
        info.modifiers.properties.bytes(),
    ) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Identity V2 policy-authority constitutional property rejected: {error:?}"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_crypto::AlgorithmId;
    use mycelix_historical_activation_time_policy_authority_bootstrap_declaration_policy::{
        canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2,
        HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
        BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
    };
    use mycelix_historical_activation_time_policy_authority_bootstrap_property_wire_policy::{
        encode_historical_activation_time_policy_authority_bootstrap_property_v2,
        HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2,
    };
    use mycelix_historical_activation_time_policy_authority_bootstrap_subject_policy::HistoricalActivationTimePolicyAuthorityScopeV2;

    static PUBLIC_KEY: [u8; 32] = [0x66; 32];

    fn full_scope() -> HistoricalActivationTimePolicyAuthorityScopeV2 {
        HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(true, true, true, true)
            .unwrap()
    }

    fn property_for(constitution_id: &'static str, constitution_version: u32) -> Vec<u8> {
        let declaration =
            canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
                HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
                    format_version: BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
                    constitution_id,
                    constitution_version,
                    policy_authority_id: "identity:policy-authority:bootstrap-v2",
                    policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                    algorithm: AlgorithmId::Ed25519,
                    public_key_bytes: &PUBLIC_KEY,
                    key_generation: 1,
                    scope: full_scope(),
                },
            )
            .unwrap();
        encode_historical_activation_time_policy_authority_bootstrap_property_v2(&declaration)
            .unwrap()
    }

    #[test]
    fn canonical_identity_v2_constitution_is_genesis_admissible() {
        let property = property_for(
            IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_ID,
            IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_VERSION,
        );
        assert_eq!(property.len(), 210);
        assert_eq!(
            validate_identity_v2_policy_authority_genesis_property(&property),
            Ok(())
        );
    }

    #[test]
    fn different_constitution_id_is_rejected_after_canonical_wire_admission() {
        let property = property_for(
            "mycelix-identity-v2-other-constitution",
            IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_VERSION,
        );
        assert_eq!(
            validate_identity_v2_policy_authority_genesis_property(&property),
            Err(IdentityV2PolicyAuthorityGenesisError::ConstitutionIdMismatch)
        );
    }

    #[test]
    fn different_constitution_version_is_rejected_after_canonical_wire_admission() {
        let property = property_for(IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_ID, 2);
        assert_eq!(
            validate_identity_v2_policy_authority_genesis_property(&property),
            Err(IdentityV2PolicyAuthorityGenesisError::ConstitutionVersionMismatch)
        );
    }

    #[test]
    fn future_envelope_version_fails_before_constitution_semantics() {
        let mut property = property_for(
            IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_ID,
            IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_VERSION,
        );
        property[11] = property[11].wrapping_add(1);
        assert_eq!(
            validate_identity_v2_policy_authority_genesis_property(&property),
            Err(IdentityV2PolicyAuthorityGenesisError::PropertyWire(
                HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnsupportedEnvelopeVersion,
            ))
        );
    }

    #[test]
    fn malformed_magic_fails_closed() {
        let mut property = property_for(
            IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_ID,
            IDENTITY_V2_POLICY_AUTHORITY_CONSTITUTION_VERSION,
        );
        property[0] ^= 0x01;
        assert_eq!(
            validate_identity_v2_policy_authority_genesis_property(&property),
            Err(IdentityV2PolicyAuthorityGenesisError::PropertyWire(
                HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::MagicMismatch,
            ))
        );
    }

    #[test]
    fn genesis_reads_only_host_committed_property_bytes_and_mints_no_authority() {
        let source = include_str!("lib.rs");
        let production = &source[..source.index("#[cfg(test)]").unwrap()];
        assert!(production.contains("let info = dna_info()?;"));
        assert!(production.contains("info.modifiers.properties.bytes(),"));
        for forbidden in [
            "info.hash",
            "get_raw_39",
            "authority_domain",
            "must_get_",
            "use hdk::",
            "SerializedBytes::",
            "UnsafeBytes",
            "from_trusted_configuration",
            "is_authorized",
            "verify_strict",
            "positive_evidence",
            "trust_score",
            "sys_time(",
            "std::time",
        ] {
            assert!(!production.contains(forbidden));
        }
    }
}
